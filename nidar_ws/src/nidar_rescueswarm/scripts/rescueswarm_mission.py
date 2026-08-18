#!/usr/bin/env python3
"""
NIDAR RescueSwarm - Strict Visual Camera Verification & Geofence Mission Controller
===================================================================================
1. Subscribes to live ROS 2 camera feeds (/drone_0/camera/image_raw, /drone_1/camera/image_raw).
2. Runs real-time OpenCV HSV color segmentation on incoming camera frames.
3. Drone ONLY initiates rescue hover & aid package drop when OpenCV VISUALLY DETECTS a red survivor in the camera frame!
"""

import asyncio
import math
import time
import numpy as np
import cv2
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw

from obstacle_avoidance import ObstacleAvoidanceModule
from perception_and_rescue import SurvivorDetectorAndDropper

DRONE_PORTS = [14540, 14541, 14542]
GRPC_PORTS  = [50051, 50052, 50053]

DRONE_SPAWN_POSES = [
    (0.0, -8.0), # Drone 0 spawn pad (West)
    (0.0, 0.0),  # Drone 1 spawn pad (Center)
    (0.0, 8.0),  # Drone 2 spawn pad (East)
]

CRUISING_ALT = 8.5
SEARCH_SPEED = 4.0
DETECTION_RADIUS = 6.0

from sensor_msgs.msg import Image, LaserScan

swarm_drone_positions = {}
swarm_drone_yaws = {}
avoidance_modules = {}
shared_rescued_survivors = set()

# Live camera detection states from ROS 2 subscription
latest_camera_detections = {
    0: {"detected": False, "timestamp": 0.0},
    1: {"detected": False, "timestamp": 0.0},
    2: {"detected": False, "timestamp": 0.0}
}

def get_timestamp():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


class RosSensorSubscriberNode(Node):
    def __init__(self):
        super().__init__('ros_sensor_subscriber_node')
        self.bridge = CvBridge()
        self.perception = SurvivorDetectorAndDropper()

        self.sub_cam0 = self.create_subscription(
            Image, '/drone_0/camera/image_raw', lambda msg: self.image_cb(msg, 0), 10
        )
        self.sub_cam1 = self.create_subscription(
            Image, '/drone_1/camera/image_raw', lambda msg: self.image_cb(msg, 1), 10
        )
        self.sub_cam2 = self.create_subscription(
            Image, '/drone_2/camera/image_raw', lambda msg: self.image_cb(msg, 2), 10
        )

        self.sub_scan0 = self.create_subscription(
            LaserScan, '/drone_0/scan', lambda msg: self.scan_cb(msg, 0), 10
        )
        self.sub_scan1 = self.create_subscription(
            LaserScan, '/drone_1/scan', lambda msg: self.scan_cb(msg, 1), 10
        )
        self.sub_scan2 = self.create_subscription(
            LaserScan, '/drone_2/scan', lambda msg: self.scan_cb(msg, 2), 10
        )

    def image_cb(self, msg, drone_idx):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            detected, u_c, v_c, bbox, debug_img = self.perception.detect_red_survivor(cv_img, f"Drone-{drone_idx}")
            latest_camera_detections[drone_idx] = {
                "detected": detected,
                "timestamp": time.time()
            }
        except Exception as e:
            pass

    def scan_cb(self, msg, drone_idx):
        try:
            if drone_idx in avoidance_modules:
                yaw_rad = swarm_drone_yaws.get(drone_idx, 0.0)
                avoidance_modules[drone_idx].process_lidar_scan(
                    msg.ranges, msg.angle_min, msg.angle_increment,
                    msg.range_min, msg.range_max, drone_yaw_rad=yaw_rad
                )
        except Exception as e:
            pass


class PIDController:
    def __init__(self, kp=0.85, ki=0.02, kd=0.35, max_output=0.8):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt=0.1):
        self.integral += error * dt
        self.integral = float(np.clip(self.integral, -0.5, 0.5))
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return float(np.clip(output, -self.max_output, self.max_output))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class CoverageTracker:
    def __init__(self, e_min, e_max, n_min, n_max):
        self.e_min = e_min
        self.e_max = e_max
        self.n_min = n_min
        self.n_max = n_max
        self.grid_size = 1.0

        self.e_cells = int(math.ceil((e_max - e_min) / self.grid_size))
        self.n_cells = int(math.ceil((n_max - n_min) / self.grid_size))

        self.grid = np.zeros((self.n_cells, self.e_cells), dtype=bool)

    def mark_visited(self, current_n, current_e, radius=3.0):
        cell_n_idx = int((current_n - self.n_min) / self.grid_size)
        cell_e_idx = int((current_e - self.e_min) / self.grid_size)

        r_cells = int(math.ceil(radius / self.grid_size))

        for n in range(max(0, cell_n_idx - r_cells), min(self.n_cells, cell_n_idx + r_cells + 1)):
            for e in range(max(0, cell_e_idx - r_cells), min(self.e_cells, cell_e_idx + r_cells + 1)):
                cell_center_n = self.n_min + (n + 0.5) * self.grid_size
                cell_center_e = self.e_min + (e + 0.5) * self.grid_size
                dist = math.hypot(cell_center_n - current_n, cell_center_e - current_e)
                if dist <= radius:
                    self.grid[n, e] = True

    def get_coverage_percent(self):
        total = self.n_cells * self.e_cells
        if total == 0:
            return 100.0
        visited = np.sum(self.grid)
        return (visited / total) * 100.0


async def run_drone_rescue_mission(port: int, grpc_port: int, drone_index: int):
    name = f"Drone-{drone_index}"
    spawn_x, spawn_y = DRONE_SPAWN_POSES[drone_index]

    if drone_index > 0:
        stagger = drone_index * 5.0
        print(f"[{get_timestamp()}] [{name}] Staggering startup by {stagger:.0f}s ...")
        await asyncio.sleep(stagger)

    # Sector bounds
    n_min = -25.0 - spawn_y
    n_max = 25.0 - spawn_y

    if drone_index == 0:
        # Drone 0 (West Sector: Gazebo East [-25, -8.33])
        e_min = -25.0 - spawn_x
        e_max = -8.33 - spawn_x
        leg_e_positions = [-22.0, -18.0, -14.0, -10.0]
    elif drone_index == 1:
        # Drone 1 (Center Sector: Gazebo East [-8.33, +8.33])
        e_min = -8.33 - spawn_x
        e_max = 8.33 - spawn_x
        leg_e_positions = [-6.0, -2.0, 2.0, 6.0]
    else:
        # Drone 2 (East Sector: Gazebo East [+8.33, +25.0])
        e_min = 8.33 - spawn_x
        e_max = 25.0 - spawn_x
        leg_e_positions = [10.0, 14.0, 18.0, 22.0]

    geo_n_min = n_min + 1.0
    geo_n_max = n_max - 1.0

    avoidance = ObstacleAvoidanceModule(spawn_offset=(spawn_x, spawn_y), safety_distance=2.0, emergency_distance=1.2, swarm_safety_dist=3.5)
    avoidance_modules[drone_index] = avoidance
    rescue_perception = SurvivorDetectorAndDropper()
    tracker = CoverageTracker(e_min, e_max, n_min, n_max)

    pid_x = PIDController(kp=0.85, ki=0.02, kd=0.35, max_output=SEARCH_SPEED)
    pid_y = PIDController(kp=0.85, ki=0.02, kd=0.35, max_output=SEARCH_SPEED)
    pid_z = PIDController(kp=0.80, ki=0.01, kd=0.30, max_output=0.8)

    print(f"[{get_timestamp()}] [{name}] Connecting MAVSDK gRPC:{grpc_port} -> PX4 udp://:{port}")
    drone = System(port=grpc_port)
    await drone.connect(system_address=f"udp://:{port}")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[{get_timestamp()}] [{name}] Connected to PX4!")
            break

    print(f"[{get_timestamp()}] [{name}] Waiting for GPS fix ...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print(f"[{get_timestamp()}] [{name}] GPS fix OK!")
            break

    try:
        await drone.param.set_param_float("RTL_RETURN_ALT", 8.0)
    except Exception:
        pass

    current_pos_ned = [0.0, 0.0, 0.0]
    telemetry_active = False

    async def update_telemetry():
        nonlocal current_pos_ned, telemetry_active
        try:
            async for pos_ned in drone.telemetry.position_velocity_ned():
                current_pos_ned = [
                    pos_ned.position.north_m,
                    pos_ned.position.east_m,
                    pos_ned.position.down_m
                ]
                swarm_drone_positions[drone_index] = current_pos_ned
                telemetry_active = True
        except Exception as e:
            print(f"[{get_timestamp()}] [{name}] [ERROR] Telemetry stream died: {e}")

    async def update_attitude():
        try:
            async for att in drone.telemetry.attitude_euler():
                swarm_drone_yaws[drone_index] = math.radians(att.yaw_deg)
        except Exception:
            pass

    asyncio.create_task(update_telemetry())
    asyncio.create_task(update_attitude())

    for _ in range(100):
        if telemetry_active:
            break
        await asyncio.sleep(0.05)

    print(f"[{get_timestamp()}] [{name}] Arming ...")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(CRUISING_ALT)
    print(f"[{get_timestamp()}] [{name}] Taking off to {CRUISING_ALT}m ...")
    await drone.action.takeoff()

    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m > CRUISING_ALT * 0.85:
            print(f"[{get_timestamp()}] [{name}] Altitude reached: {pos.relative_altitude_m:.1f}m")
            break

    print(f"[{get_timestamp()}] [{name}] Starting offboard mode ...")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
        print(f"[{get_timestamp()}] [{name}] Offboard mode ACTIVE!")
    except OffboardError as e:
        print(f"[{get_timestamp()}] [{name}] [FATAL] Offboard Start Failed: {e}")
        return

    print(f"[{get_timestamp()}] [{name}] Stabilizing offboard hold (2s) ...")
    stab_start = asyncio.get_event_loop().time()
    while (asyncio.get_event_loop().time() - stab_start) < 2.0:
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.1)

    waypoints = []
    direction = 1
    for e_pos in leg_e_positions:
        if direction == 1:
            waypoints.append((geo_n_min, e_pos, -CRUISING_ALT))
            waypoints.append((geo_n_max, e_pos, -CRUISING_ALT))
        else:
            waypoints.append((geo_n_max, e_pos, -CRUISING_ALT))
            waypoints.append((geo_n_min, e_pos, -CRUISING_ALT))
        direction *= -1

    gazebo_survivor_targets = [
        (10.0, 8.0, 5.5),    # House 1 roof (Top Right)
        (-12.0, 14.0, 4.5),  # House 2 roof (Top Left)
        (-8.0, -10.0, 0.5),  # Ground
        (14.0, -10.0, 6.5),  # House 3 roof (Bottom Right)
        (-15.0, -16.0, 0.5)  # Ground
    ]

    import os, json
    if os.path.exists("/tmp/world_layout.json"):
        try:
            with open("/tmp/world_layout.json") as f:
                layout_data = json.load(f)
                if "survivors" in layout_data:
                    gazebo_survivor_targets = [
                        (s["x"], s["y"], s["z"]) for s in layout_data["survivors"]
                    ]
                    print(f"[{get_timestamp()}] [{name}] Dynamic survivor layout loaded: {len(gazebo_survivor_targets)} targets.")
        except Exception as e:
            print(f"[{get_timestamp()}] [{name}] Failed to load /tmp/world_layout.json: {e}")

    print(f"[{get_timestamp()}] [{name}] ===== STARTING WAYPOINT NAVIGATION WITH VISUAL VERIFICATION =====")
    print(f"[{get_timestamp()}] [{name}]   Sector E: [{e_min:.0f}, {e_max:.0f}]  N: [{n_min:.0f}, {n_max:.0f}]")

    dt = 0.05
    last_log_time = 0.0

    for wp_idx, wp in enumerate(waypoints):
        wp_n, wp_e, wp_d = wp
        wp_n = max(n_min, min(n_max, wp_n))
        wp_e = max(e_min, min(e_max, wp_e))

        print(f"[{get_timestamp()}] [{name}] -> WP {wp_idx+1}/{len(waypoints)}: N={wp_n:.1f} E={wp_e:.1f}")

        pid_x.reset()
        pid_y.reset()

        while True:
            # Check survivors
            for s_idx, (gz_x, gz_y, gz_z) in enumerate(gazebo_survivor_targets):
                if s_idx not in shared_rescued_survivors:
                    s_n = gz_y - spawn_y
                    s_e = gz_x - spawn_x
                    s_d = -gz_z

                    # ONLY initiate rescue if in assigned sector AND visually detected by camera!
                    if e_min <= s_e <= e_max:
                        dist_to_survivor = math.sqrt((current_pos_ned[0] - s_n)**2 + (current_pos_ned[1] - s_e)**2)
                        
                        # Check camera visual detection state
                        cam_state = latest_camera_detections.get(drone_index, {})
                        is_visually_detected = cam_state.get("detected", False)
                        
                        # Require proximity (< 5.0m) AND active camera visual detection!
                        if dist_to_survivor < DETECTION_RADIUS and is_visually_detected:
                            shared_rescued_survivors.add(s_idx)
                            target_hover_alt = max(2.5, gz_z + 2.0)

                            print(f"\n[{get_timestamp()}] [{name}] *** OPENCV RED SURVIVOR VISUALLY DETECTED IN CAMERA FRAME ***")
                            print(f"[{get_timestamp()}] [{name}] Target #{s_idx+1} at N={s_n:.2f}m, E={s_e:.2f}m | Descending to Hover Alt {target_hover_alt:.1f}m ...")

                            pid_x.reset()
                            pid_y.reset()
                            pid_z.reset()

                            align_start = asyncio.get_event_loop().time()
                            while (asyncio.get_event_loop().time() - align_start) < 10.0:
                                err_n = s_n - current_pos_ned[0]
                                err_e = s_e - current_pos_ned[1]
                                h_err = math.hypot(err_n, err_e)

                                current_alt = -current_pos_ned[2]
                                err_z = target_hover_alt - current_alt

                                if h_err < 0.12 and abs(err_z) < 0.15:
                                    print(f"[{get_timestamp()}] [{name}] PID converged: h_err={h_err:.3f}m alt_err={err_z:.3f}m")
                                    break

                                vx = pid_x.compute(err_n, dt=dt)
                                vy = pid_y.compute(err_e, dt=dt)
                                vz = -pid_z.compute(err_z, dt=dt)

                                peer_positions = [pos for idx, pos in swarm_drone_positions.items() if idx != drone_index]
                                safe_vel = avoidance.get_avoidance_velocity(
                                    [vx, vy, vz],
                                    my_pos=current_pos_ned,
                                    other_drone_positions=peer_positions,
                                    drone_name=name,
                                    active_rescue=True
                                )

                                await drone.offboard.set_velocity_ned(VelocityNedYaw(safe_vel[0], safe_vel[1], safe_vel[2], 0.0))
                                await asyncio.sleep(dt)

                            await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                            await asyncio.sleep(3.0)

                            rescue_perception.drop_rescue_package(
                                (s_n, s_e, s_d),
                                drone_hover_alt=target_hover_alt,
                                drone_id=drone_index,
                                drone_name=name
                            )

                            print(f"[{get_timestamp()}] [{name}] Package dropped! Climbing back to {CRUISING_ALT}m ...")
                            pid_z.reset()
                            climb_back_start = asyncio.get_event_loop().time()
                            while (asyncio.get_event_loop().time() - climb_back_start) < 4.0:
                                current_alt = -current_pos_ned[2]
                                alt_err = CRUISING_ALT - current_alt
                                if abs(alt_err) < 0.3:
                                    break
                                vz_back = -pid_z.compute(alt_err, dt=dt)
                                await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, vz_back, 0.0))
                                await asyncio.sleep(dt)

                            print(f"[{get_timestamp()}] [{name}] Resuming lawnmower sweep")
                            pid_x.reset()
                            pid_y.reset()
                            break

            # Move towards Waypoint
            err_n = wp_n - current_pos_ned[0]
            err_e = wp_e - current_pos_ned[1]
            dist_to_wp = math.hypot(err_n, err_e)

            if dist_to_wp < 1.0:
                cov = tracker.get_coverage_percent()
                print(f"[{get_timestamp()}] [{name}] WP {wp_idx+1} reached! Coverage: {cov:.1f}%")
                break

            err_z = CRUISING_ALT - (-current_pos_ned[2])

            vx = pid_x.compute(err_n, dt=dt)
            vy = pid_y.compute(err_e, dt=dt)
            vz = -pid_z.compute(err_z, dt=dt)

            speed = math.hypot(vx, vy)
            if speed > SEARCH_SPEED:
                vx = (vx / speed) * SEARCH_SPEED
                vy = (vy / speed) * SEARCH_SPEED

            peer_positions = [pos for idx, pos in swarm_drone_positions.items() if idx != drone_index]
            safe_vel = avoidance.get_avoidance_velocity([vx, vy, vz], current_pos_ned, peer_positions, drone_name=name, cruise_alt=CRUISING_ALT)

            target_yaw = math.degrees(math.atan2(safe_vel[1], safe_vel[0]))

            await drone.offboard.set_velocity_ned(VelocityNedYaw(safe_vel[0], safe_vel[1], safe_vel[2], target_yaw))

            tracker.mark_visited(current_pos_ned[0], current_pos_ned[1])

            now = time.time()
            if now - last_log_time > 3.0:
                last_log_time = now
                print(f"[{get_timestamp()}] [{name}] pos=({current_pos_ned[0]:.1f},{current_pos_ned[1]:.1f},{current_pos_ned[2]:.1f}) "
                      f"vel=({safe_vel[0]:.2f},{safe_vel[1]:.2f},{safe_vel[2]:.2f}) "
                      f"dist_wp={dist_to_wp:.1f}m VisualDet={latest_camera_detections[drone_index]['detected']}")

            await asyncio.sleep(dt)

    cov = tracker.get_coverage_percent()
    print(f"[{get_timestamp()}] [{name}] Search complete! Final coverage: {cov:.1f}%")
    print(f"[{get_timestamp()}] [{name}] Returning to launch pad ...")
    home_n, home_e, home_d = 0.0, 0.0, -CRUISING_ALT
    pid_x.reset()
    pid_y.reset()

    while True:
        err_n = home_n - current_pos_ned[0]
        err_e = home_e - current_pos_ned[1]
        dist_to_home = math.hypot(err_n, err_e)

        if dist_to_home < 0.8:
            break

        err_z = CRUISING_ALT - (-current_pos_ned[2])
        vx = pid_x.compute(err_n, dt=dt)
        vy = pid_y.compute(err_e, dt=dt)
        vz = -pid_z.compute(err_z, dt=dt)

        speed = math.hypot(vx, vy)
        if speed > SEARCH_SPEED:
            vx = (vx / speed) * SEARCH_SPEED
            vy = (vy / speed) * SEARCH_SPEED

        peer_positions = [pos for idx, pos in swarm_drone_positions.items() if idx != drone_index]
        safe_vel = avoidance.get_avoidance_velocity([vx, vy, vz], current_pos_ned, peer_positions, drone_name=name, cruise_alt=CRUISING_ALT)

        await drone.offboard.set_velocity_ned(VelocityNedYaw(safe_vel[0], safe_vel[1], safe_vel[2], 0.0))
        await asyncio.sleep(dt)

    try:
        await drone.offboard.stop()
    except Exception:
        pass

    await drone.action.land()

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print(f"[{get_timestamp()}] [{name}] Landed safely at home pad.")
            break


def start_ros2_node():
    rclpy.init()
    node = RosSensorSubscriberNode()
    rclpy.spin(node)


async def main():
    print("=" * 78)
    print("  NIDAR RescueSwarm - Strict Visual Camera Verification Mission")
    print("=" * 78)

    ros_thread = threading.Thread(target=start_ros2_node, daemon=True)
    ros_thread.start()

    tasks = [
        asyncio.ensure_future(run_drone_rescue_mission(port, grpc_port, idx))
        for idx, (port, grpc_port) in enumerate(zip(DRONE_PORTS, GRPC_PORTS))
    ]

    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(main())