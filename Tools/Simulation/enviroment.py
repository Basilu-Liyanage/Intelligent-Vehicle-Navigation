#!/usr/bin/env python3
"""
TESLA-BEATING ENVIRONMENT
Full Bird-Eye & Bat-Eye simulation with HD maps
"""

import numpy as np
import gymnasium as gym
import math
from dataclasses import dataclass
from typing import Tuple, List

@dataclass
class BirdEyeData:
    """Bird-Eye: GPS + IMU data"""
    lat: float
    lon: float
    velocity_x: float
    velocity_y: float
    yaw: float
    pitch: float
    roll: float
    altitude: float

@dataclass  
class BatEyeData:
    """Bat-Eye: LiDAR scanning data"""
    distances: List[float]  # Multiple distance readings
    angles: List[float]     # Corresponding angles
    quality: float          # Signal quality

class TrainingEnviroment(gym.Env):
    """
    REALISTIC autonomous driving environment
    NO CAMERAS - Pure geometric navigation
    """
    
    def __init__(self, hd_map=None):
        super().__init__()
        
        # ===== OBSERVATION SPACE =====
        # [0-9]: Same as your 302-reward training (for compatibility)
        # [10-19]: Bird-Eye data (GPS/IMU)
        # [20-35]: Bat-Eye scan (16-point LiDAR sweep)
        self.observation_space = gym.spaces.Box(
            low=np.concatenate([
                np.array([0, -5, -5, -math.pi, -1, -1, 0, -1, 0, 0]),  # Original 10
                np.zeros(10),  # Bird-Eye placeholder
                np.zeros(16),  # Bat-Eye scan
                np.zeros(1)    # Eye position
            ]),
            high=np.concatenate([
                np.array([50, 5, 5, math.pi, 1, 1, 1, 1, 1, 10]),  # Original 10
                np.ones(10) * 100,   # Bird-Eye (GPS coords can be large)
                np.ones(16) * 50,    # Bat-Eye max distance 50m
                np.ones(1)           # Eye position
            ]),
            dtype=np.float32
        )
        
        # ===== ACTION SPACE =====
        # [steering, throttle, eye_angle]
        self.action_space = gym.spaces.Box(
            low=np.array([-1.0, 0.0, -1.0]),
            high=np.array([1.0, 1.0, 1.0]),
            dtype=np.float32
        )
        
        # ===== HD MAP (Bird-Eye) =====
        self.hd_map = hd_map or self._create_default_map()
        
        # ===== LiDAR SCANNER (Bat-Eye) =====
        self.lidar_fov = 60  # degrees
        self.lidar_resolution = 16  # 16-point scan
        
        # ===== VEHICLE PARAMS =====
        self.vehicle_length = 0.5  # meters
        self.max_speed = 5.0  # m/s
        self.eye_servo_range = (-30, 30)  # degrees
        
        # ===== TRAINING PARAMS =====
        self.max_steps = 1000
        self.crash_distance = 0.3
        self.target_speed = 2.0  # Optimal speed
        
        print("=" * 70)
        print("🚗 TESLA-BEATING AUTONOMOUS VEHICLE ENVIRONMENT")
        print("=" * 70)
        print(f"Observation: {self.observation_space.shape[0]} dimensions")
        print(f"  • [0-9]: Base navigation")
        print(f"  • [10-19]: Bird-Eye (GPS/IMU)")
        print(f"  • [20-35]: Bat-Eye (16-point LiDAR scan)")
        print(f"  • [36]: Eye servo position")
        print(f"Action: [steering, throttle, eye_angle]")
        print("=" * 70)
    
    def _create_default_map(self):
        """Create a simple HD map for training"""
        return {
            'waypoints': [
                (0, 0), (10, 0), (20, 5), (30, 5), (40, 0), (50, 0)
            ],
            'obstacles': [
                (15, -2, 2, 1),  # (x, y, width, height)
                (25, 2, 3, 1),
                (35, -1, 2, 1)
            ],
            'boundaries': [(-5, 5)] * 100  # Road width
        }
    
    def reset(self, seed=None):
        super().reset(seed=seed)
        
        # Reset vehicle state
        self.step_count = 0
        self.position = np.array([0.0, 0.0])  # (x, y)
        self.velocity = np.array([0.0, 0.0])
        self.yaw = 0.0
        self.eye_angle = 0.0  # degrees
        
        # Reset Bird-Eye
        self.bird_eye = BirdEyeData(
            lat=0.0, lon=0.0,
            velocity_x=0.0, velocity_y=0.0,
            yaw=0.0, pitch=0.0, roll=0.0,
            altitude=0.0
        )
        
        # Reset Bat-Eye
        self.bat_eye = BatEyeData(
            distances=[10.0] * self.lidar_resolution,
            angles=np.linspace(-self.lidar_fov/2, self.lidar_fov/2, self.lidar_resolution),
            quality=1.0
        )
        
        # Reset trajectory
        self.trajectory = [self.position.copy()]
        self.last_actions = [(0, 0, 0), (0, 0, 0)]
        
        return self._get_obs(), {}
    
    def _get_obs(self):
        """Build complete observation vector"""
        
        # ===== 1. BASE NAVIGATION (10D - same as your 302-reward) =====
        base_obs = np.zeros(10, dtype=np.float32)
        
        # Forward distance (simplified)
        forward_dist = self._get_forward_distance()
        base_obs[0] = forward_dist / 50.0  # Normalized
        
        # Velocity
        base_obs[1] = self.velocity[0] / self.max_speed  # X velocity
        base_obs[2] = self.velocity[1] / self.max_speed  # Y velocity
        
        # Yaw
        base_obs[3] = self.yaw / math.pi  # Normalized
        
        # Eye position (NEW - at position 4 instead of separate)
        base_obs[4] = self.eye_angle / 30.0  # Normalized -1 to 1
        
        # Last actions
        if len(self.last_actions) >= 1:
            base_obs[5] = self.last_actions[-1][0]  # Last steering
            base_obs[6] = self.last_actions[-1][1]  # Last throttle
        
        if len(self.last_actions) >= 2:
            base_obs[7] = self.last_actions[-2][0]  # Second last steering
            base_obs[8] = self.last_actions[-2][1]  # Second last throttle
        
        # Progress
        base_obs[9] = self.position[0] / 100.0  # Normalized progress
        
        # ===== 2. BIRD-EYE DATA (10D) =====
        bird_eye_obs = np.array([
            self.bird_eye.lat / 1000.0,
            self.bird_eye.lon / 1000.0,
            self.bird_eye.velocity_x / self.max_speed,
            self.bird_eye.velocity_y / self.max_speed,
            self.bird_eye.yaw / math.pi,
            self.bird_eye.pitch / math.pi,
            self.bird_eye.roll / math.pi,
            self.bird_eye.altitude / 100.0,
            self.position[0] / 100.0,  # X progress
            self.position[1] / 10.0    # Y offset
        ], dtype=np.float32)
        
        # ===== 3. BAT-EYE SCAN (16D) =====
        bat_eye_obs = np.array(self.bat_eye.distances, dtype=np.float32) / 50.0
        
        # ===== 4. EYE POSITION (1D) - Duplicate for emphasis =====
        eye_obs = np.array([self.eye_angle / 30.0], dtype=np.float32)
        
        # ===== COMBINE =====
        full_obs = np.concatenate([base_obs, bird_eye_obs, bat_eye_obs, eye_obs])
        
        return full_obs
    
    def step(self, action):
        steering, throttle, eye_command = action
        self.step_count += 1
        
        # ===== 1. UPDATE EYE SERVO =====
        self.eye_angle = np.clip(eye_command * 30.0, -30, 30)
        
        # ===== 2. UPDATE VEHICLE DYNAMICS =====
        # Convert throttle to acceleration
        acceleration = throttle * 3.0  # m/s²
        
        # Update velocity
        speed = np.linalg.norm(self.velocity)
        if speed < self.max_speed:
            forward_dir = np.array([math.cos(self.yaw), math.sin(self.yaw)])
            self.velocity += forward_dir * acceleration * 0.1
        
        # Update position
        self.position += self.velocity * 0.1
        
        # Update yaw (steering)
        steering_force = steering * 0.5  # Max 0.5 rad/s
        self.yaw += steering_force * 0.1
        
        # ===== 3. UPDATE BIRD-EYE (GPS/IMU simulation) =====
        self.bird_eye.lat = self.position[0] * 1e-5  # Fake lat/lon
        self.bird_eye.lon = self.position[1] * 1e-5
        self.bird_eye.velocity_x = self.velocity[0]
        self.bird_eye.velocity_y = self.velocity[1]
        self.bird_eye.yaw = self.yaw
        self.bird_eye.pitch = 0.0  # Flat terrain
        self.bird_eye.roll = steering * 0.1  # Simulate roll from turning
        
        # ===== 4. UPDATE BAT-EYE (LiDAR scan) =====
        self._update_lidar_scan()
        
        # ===== 5. STORE ACTION HISTORY =====
        self.last_actions.append((steering, throttle, eye_command))
        if len(self.last_actions) > 2:
            self.last_actions.pop(0)
        
        # ===== 6. CALCULATE REWARD =====
        reward = self._calculate_reward(steering, throttle, eye_command)
        
        # ===== 7. CHECK TERMINATION =====
        terminated = self._check_collision()
        truncated = self.step_count >= self.max_steps
        
        # ===== 8. UPDATE TRAJECTORY =====
        self.trajectory.append(self.position.copy())
        
        return self._get_obs(), reward, terminated, truncated, {}
    
    def _update_lidar_scan(self):
        """Simulate LiDAR scanning based on eye angle"""
        # Base angles relative to vehicle heading
        base_angles = np.linspace(
            -self.lidar_fov/2, 
            self.lidar_fov/2, 
            self.lidar_resolution
        )
        
        # Adjust for eye angle
        adjusted_angles = base_angles + self.eye_angle
        
        # Simulate distance measurements
        distances = []
        for angle in adjusted_angles:
            # Ray casting simulation (simplified)
            look_angle = self.yaw + math.radians(angle)
            look_dir = np.array([math.cos(look_angle), math.sin(look_angle)])
            
            # Check HD map for obstacles
            distance = self._ray_cast(self.position, look_dir)
            distances.append(distance)
        
        self.bat_eye.distances = distances
        self.bat_eye.angles = adjusted_angles.tolist()
    
    def _ray_cast(self, origin, direction, max_dist=50.0):
        """Simple ray casting for obstacle detection"""
        # Check map obstacles
        for obs in self.hd_map['obstacles']:
            obs_x, obs_y, obs_w, obs_h = obs
            # Simple AABB collision check
            # (In reality, you'd do proper ray-box intersection)
            pass
        
        # Default: return some distance with noise
        base_dist = 10.0 + np.random.uniform(-2, 2)
        return min(base_dist, max_dist)
    
    def _calculate_reward(self, steering, throttle, eye_command):
        """Tesla-beating reward function"""
        reward = 0.0
        
        # 1. FORWARD PROGRESS (Primary objective)
        speed = np.linalg.norm(self.velocity)
        progress_reward = speed * 0.5
        reward += progress_reward
        
        # 2. CRASH AVOIDANCE (Critical)
        min_distance = min(self.bat_eye.distances) if self.bat_eye.distances else 10.0
        if min_distance < self.crash_distance:
            reward -= 20.0  # Severe crash penalty
        elif min_distance < 1.0:
            safety_penalty = (1.0 - min_distance) * 5.0
            reward -= safety_penalty
        
        # 3. SPEED OPTIMIZATION (Tesla-like smoothness)
        speed_error = abs(speed - self.target_speed)
        reward -= speed_error * 0.1
        
        # 4. STEERING SMOOTHNESS
        if len(self.last_actions) >= 2:
            last_steering = self.last_actions[-2][0]
            steering_change = abs(steering - last_steering)
            reward -= steering_change * 0.05
        
        # 5. EYE ALIGNMENT BONUS (Looking where you're going)
        eye_alignment = 1.0 - abs(steering - eye_command) / 2.0
        reward += eye_alignment * 0.3
        
        # 6. BAT-EYE UTILIZATION BONUS (Active scanning)
        if abs(eye_command) > 0.3:  # Actively looking around
            reward += 0.1
        
        # 7. BIRD-EYE PATH FOLLOWING (GPS waypoints)
        waypoint_dist = self._distance_to_nearest_waypoint()
        if waypoint_dist < 2.0:
            reward += 0.2  # Close to waypoint bonus
        
        return reward
    
    def _check_collision(self):
        """Check if vehicle has collided"""
        min_distance = min(self.bat_eye.distances) if self.bat_eye.distances else 10.0
        return min_distance < self.crash_distance
    
    def _get_forward_distance(self):
        """Distance directly ahead"""
        forward_angle = self.yaw + math.radians(self.eye_angle)
        forward_dir = np.array([math.cos(forward_angle), math.sin(forward_angle)])
        return self._ray_cast(self.position, forward_dir)
    
    def _distance_to_nearest_waypoint(self):
        """Distance to nearest HD map waypoint"""
        if not self.hd_map['waypoints']:
            return 0.0
        
        distances = [np.linalg.norm(self.position - np.array(wp)) 
                    for wp in self.hd_map['waypoints']]
        return min(distances)
    
    def render(self):
        """Text-based rendering"""
        progress = self.position[0]
        speed = np.linalg.norm(self.velocity)
        min_dist = min(self.bat_eye.distances) if self.bat_eye.distances else 10.0
        
        print(f"Step: {self.step_count:4d} | "
              f"Pos: ({self.position[0]:5.1f}, {self.position[1]:5.1f}) | "
              f"Speed: {speed:4.2f}m/s | "
              f"Eye: {self.eye_angle:5.1f}° | "
              f"Min Dist: {min_dist:4.2f}m")