#!/usr/bin/env python3
"""
SPATIAL AI TRAINER FOR SCANNING LIDAR - NO MATPLOTLIB
Trains AI to understand servo-LiDAR relationship AND steering
"""

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import time
import json
import random
import os
import sys
from collections import deque

class SpatialRLAgent:
    """
    Reinforcement Learning Agent that learns:
    1. Servo position affects LiDAR readings
    2. How to steer based on spatial understanding
    3. Speed control based on obstacle distances
    """
    
    def __init__(self, state_dim=6, action_dim=3, hidden_dim=128):
        """
        state_dim: [servo_pos, lidar_dist, motor_speed, heading, step, scan_mode]
        action_dim: [eye_servo_delta, driver_servo_delta, motor_speed]
        """
        self.state_dim = state_dim
        self.action_dim = action_dim
        
        # Actor Network (Policy)
        self.actor = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim * 2)  # Mean + LogStd for each action
        )
        
        # Critic Networks (Q-values)
        self.critic1 = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )
        
        self.critic2 = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )
        
        # Target Networks
        self.target_critic1 = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )
        
        self.target_critic2 = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )
        
        # Initialize target networks with same weights
        self.target_critic1.load_state_dict(self.critic1.state_dict())
        self.target_critic2.load_state_dict(self.critic2.state_dict())
        
        # Optimizers
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=3e-4)
        self.critic1_optimizer = optim.Adam(self.critic1.parameters(), lr=3e-4)
        self.critic2_optimizer = optim.Adam(self.critic2.parameters(), lr=3e-4)
        
        # Experience Replay
        self.replay_buffer = deque(maxlen=50000)
        self.batch_size = 128
        self.gamma = 0.99
        self.tau = 0.005
        
        # Training stats
        self.training_step = 0
        self.episode_rewards = []
        
        print(f"✅ Spatial RL Agent initialized: State={state_dim}, Actions={action_dim}")
    
    def get_action(self, state, deterministic=False, exploration_noise=0.1):
        """Get action from policy with optional exploration"""
        state_tensor = torch.FloatTensor(state).unsqueeze(0)
        
        with torch.no_grad():
            actor_output = self.actor(state_tensor)
            mean = actor_output[:, :self.action_dim]
            log_std = actor_output[:, self.action_dim:]
            
            if deterministic:
                action = torch.tanh(mean)
            else:
                std = torch.exp(log_std)
                normal = torch.distributions.Normal(mean, std)
                action = torch.tanh(normal.sample())
                
                # Add exploration noise
                noise = torch.randn_like(action) * exploration_noise
                action = torch.clamp(action + noise, -1.0, 1.0)
        
        action_np = action.squeeze().numpy()
        
        # Scale actions to appropriate ranges:
        # action[0]: Eye servo delta (-1 to 1 maps to -25 to +25 units)
        # action[1]: Driver servo delta (-1 to 1 maps to -45 to +45 degrees)
        # action[2]: Motor speed (-1 to 1 maps to 0 to 100%)
        
        return action_np
    
    def store_experience(self, state, action, reward, next_state, done):
        """Store experience in replay buffer"""
        experience = (state, action, reward, next_state, done)
        self.replay_buffer.append(experience)
    
    def update(self):
        """Update networks using SAC algorithm"""
        if len(self.replay_buffer) < self.batch_size:
            return 0, 0, 0
        
        # Sample batch
        indices = np.random.choice(len(self.replay_buffer), self.batch_size, replace=False)
        batch = [self.replay_buffer[i] for i in indices]
        
        states, actions, rewards, next_states, dones = zip(*batch)
        
        # Convert to tensors
        states = torch.FloatTensor(states)
        actions = torch.FloatTensor(actions)
        rewards = torch.FloatTensor(rewards).unsqueeze(1)
        next_states = torch.FloatTensor(next_states)
        dones = torch.FloatTensor(dones).unsqueeze(1)
        
        # --- Critic Update ---
        with torch.no_grad():
            # Get next action from target policy
            next_actor_output = self.actor(next_states)
            next_mean = next_actor_output[:, :self.action_dim]
            next_log_std = next_actor_output[:, self.action_dim:]
            next_std = torch.exp(next_log_std)
            next_normal = torch.distributions.Normal(next_mean, next_std)
            next_actions = torch.tanh(next_normal.rsample())
            
            # Compute target Q-values
            target_q1 = self.target_critic1(torch.cat([next_states, next_actions], 1))
            target_q2 = self.target_critic2(torch.cat([next_states, next_actions], 1))
            target_q = torch.min(target_q1, target_q2)
            target_value = rewards + self.gamma * (1 - dones) * target_q
        
        # Current Q-values
        current_q1 = self.critic1(torch.cat([states, actions], 1))
        current_q2 = self.critic2(torch.cat([states, actions], 1))
        
        # Critic losses
        critic1_loss = nn.MSELoss()(current_q1, target_value)
        critic2_loss = nn.MSELoss()(current_q2, target_value)
        
        # Update critics
        self.critic1_optimizer.zero_grad()
        critic1_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic1.parameters(), 1.0)
        self.critic1_optimizer.step()
        
        self.critic2_optimizer.zero_grad()
        critic2_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic2.parameters(), 1.0)
        self.critic2_optimizer.step()
        
        # --- Actor Update ---
        actor_output = self.actor(states)
        mean = actor_output[:, :self.action_dim]
        log_std = actor_output[:, self.action_dim:]
        std = torch.exp(log_std)
        normal = torch.distributions.Normal(mean, std)
        new_actions = torch.tanh(normal.rsample())
        
        # Compute actor loss
        q1_new = self.critic1(torch.cat([states, new_actions], 1))
        q2_new = self.critic2(torch.cat([states, new_actions], 1))
        q_new = torch.min(q1_new, q2_new)
        
        # Entropy regularization
        log_prob = normal.log_prob(mean)
        log_prob -= torch.log(1 - new_actions.pow(2) + 1e-6)
        log_prob = log_prob.sum(1, keepdim=True)
        
        actor_loss = (0.2 * log_prob - q_new).mean()
        
        # Update actor
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 1.0)
        self.actor_optimizer.step()
        
        # --- Soft Update Target Networks ---
        for target_param, param in zip(self.target_critic1.parameters(), self.critic1.parameters()):
            target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)
        
        for target_param, param in zip(self.target_critic2.parameters(), self.critic2.parameters()):
            target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)
        
        self.training_step += 1
        
        return critic1_loss.item(), critic2_loss.item(), actor_loss.item()
    
    def save(self, path):
        """Save model weights"""
        torch.save({
            'actor': self.actor.state_dict(),
            'critic1': self.critic1.state_dict(),
            'critic2': self.critic2.state_dict(),
            'target_critic1': self.target_critic1.state_dict(),
            'target_critic2': self.target_critic2.state_dict(),
            'actor_optimizer': self.actor_optimizer.state_dict(),
            'critic1_optimizer': self.critic1_optimizer.state_dict(),
            'critic2_optimizer': self.critic2_optimizer.state_dict(),
            'training_step': self.training_step,
            'episode_rewards': self.episode_rewards
        }, path)
        print(f"✅ Saved model to {path}")
    
    def load(self, path):
        """Load model weights"""
        if os.path.exists(path):
            checkpoint = torch.load(path, map_location='cpu')
            self.actor.load_state_dict(checkpoint['actor'])
            self.critic1.load_state_dict(checkpoint['critic1'])
            self.critic2.load_state_dict(checkpoint['critic2'])
            self.target_critic1.load_state_dict(checkpoint['target_critic1'])
            self.target_critic2.load_state_dict(checkpoint['target_critic2'])
            self.actor_optimizer.load_state_dict(checkpoint['actor_optimizer'])
            self.critic1_optimizer.load_state_dict(checkpoint['critic1_optimizer'])
            self.critic2_optimizer.load_state_dict(checkpoint['critic2_optimizer'])
            self.training_step = checkpoint['training_step']
            self.episode_rewards = checkpoint.get('episode_rewards', [])
            print(f"✅ Loaded model from {path}")
        else:
            print(f"⚠️ No model found at {path}")

class SpatialTrainingEnv:
    """
    Training Environment that simulates servo-LiDAR relationship
    """
    
    def __init__(self):
        # Servo parameters
        self.eye_servo_range = (0, 50)  # Eye servo units (LiDAR scanner)
        self.driver_servo_range = (0, 180)  # Driver servo degrees (steering)
        
        # Current state
        self.eye_servo = 25  # Center
        self.driver_servo = 90  # Center
        self.lidar_distance = 100
        self.motor_speed = 0
        self.heading = 0
        self.position = [0, 0]  # x, y position
        
        # Environment obstacles
        self.obstacles = []
        self._generate_obstacles()
        
        # Episode tracking
        self.step_count = 0
        self.max_steps = 200
        self.total_reward = 0
        
        print("✅ Spatial Training Environment initialized")
    
    def _generate_obstacles(self):
        """Generate random obstacles for training"""
        self.obstacles = []
        for _ in range(5):
            angle = random.uniform(-30, 30)  # Relative to center
            distance = random.uniform(30, 200)
            width = random.uniform(5, 20)  # Obstacle width in degrees
            self.obstacles.append({
                'angle': angle,
                'distance': distance,
                'width': width
            })
    
    def reset(self):
        """Reset environment for new episode"""
        self.eye_servo = 25
        self.driver_servo = 90
        self.lidar_distance = self._simulate_lidar(self.eye_servo)
        self.motor_speed = 0
        self.heading = 0
        self.position = [0, 0]
        self.step_count = 0
        self.total_reward = 0
        
        # Regenerate obstacles
        self._generate_obstacles()
        
        return self._get_state()
    
    def _simulate_lidar(self, eye_servo_pos):
        """
        Simulate LiDAR reading at given eye servo position
        Models the real servo-LiDAR relationship
        """
        # Convert servo position to viewing angle (-25 to +25 degrees)
        view_angle = eye_servo_pos - 25
        
        # Base distance (farther when looking straight)
        base_distance = 200 - abs(view_angle) * 2
        
        # Check obstacles
        for obstacle in self.obstacles:
            angle_diff = abs(view_angle - obstacle['angle'])
            
            # If looking at obstacle
            if angle_diff < obstacle['width']:
                # Gaussian influence
                influence = obstacle['distance'] * np.exp(-(angle_diff**2) / (obstacle['width']**2))
                base_distance = min(base_distance, influence)
        
        # Add sensor noise
        noise = random.uniform(-10, 10)
        distance = max(10, min(1200, base_distance + noise))
        
        return distance
    
    def _get_state(self):
        """Get current state vector"""
        return np.array([
            self.eye_servo / 50.0,           # Normalized eye servo
            self.lidar_distance / 1200.0,    # Normalized LiDAR distance
            self.driver_servo / 180.0,       # Normalized driver servo
            self.motor_speed / 100.0,        # Normalized motor speed
            self.heading / 360.0,            # Normalized heading
            self.step_count / self.max_steps # Progress
        ])
    
    def step(self, action):
        """
        Execute action and return next state, reward, done
        action: [eye_servo_delta, driver_servo_delta, motor_speed]
        """
        self.step_count += 1
        
        # Parse and apply action
        eye_delta = action[0] * 12.5  # Scale to ±12.5 servo units
        driver_delta = action[1] * 45  # Scale to ±45 degrees
        target_speed = (action[2] + 1) / 2 * 100  # Convert to 0-100%
        
        # Update eye servo (LiDAR scanner)
        old_eye_servo = self.eye_servo
        self.eye_servo = max(0, min(50, old_eye_servo + eye_delta))
        
        # Update driver servo (steering)
        old_driver_servo = self.driver_servo
        self.driver_servo = max(0, min(180, old_driver_servo + driver_delta))
        
        # Update LiDAR reading based on new eye position
        old_distance = self.lidar_distance
        self.lidar_distance = self._simulate_lidar(self.eye_servo)
        
        # Update motor speed (with acceleration limit)
        speed_change = target_speed - self.motor_speed
        max_change = 20  # Maximum speed change per step
        if abs(speed_change) > max_change:
            speed_change = max_change if speed_change > 0 else -max_change
        self.motor_speed += speed_change
        self.motor_speed = max(0, min(100, self.motor_speed))
        
        # Update position and heading based on movement
        self._update_position()
        
        # Calculate reward
        reward = self._calculate_reward(
            old_eye_servo, old_driver_servo, old_distance,
            eye_delta, driver_delta
        )
        self.total_reward += reward
        
        # Check termination
        done = self.step_count >= self.max_steps
        if self.lidar_distance < 15:  # Collision
            done = True
            reward -= 10  # Big penalty for collision
        
        return self._get_state(), reward, done
    
    def _update_position(self):
        """Update vehicle position based on current speed and steering"""
        # Convert driver servo to steering angle (-45 to +45 degrees)
        steering_angle = (self.driver_servo - 90) * 0.5  # Scale down
        
        # Update heading based on steering and speed
        turn_rate = steering_angle * (self.motor_speed / 100.0) * 0.5
        self.heading = (self.heading + turn_rate) % 360
        
        # Update position (simplified)
        speed_cm_per_step = self.motor_speed * 0.5  # 0.5 cm per % per step
        rad = np.radians(self.heading)
        self.position[0] += speed_cm_per_step * np.cos(rad)
        self.position[1] += speed_cm_per_step * np.sin(rad)
    
    def _calculate_reward(self, old_eye, old_driver, old_dist, eye_delta, driver_delta):
        """Calculate reward for learning spatial relationships"""
        reward = 0
        
        # 1. SERVO-LIDAR RELATIONSHIP LEARNING REWARDS
        
        # Reward for discovering that moving eye changes LiDAR readings
        if abs(eye_delta) > 2:  # If eye moved significantly
            dist_change = abs(self.lidar_distance - old_dist)
            if dist_change > 20:  # And distance changed a lot
                reward += 3.0  # HIGH reward: Learned servo affects LiDAR!
            elif dist_change > 5:
                reward += 1.0
        
        # Reward for scanning (exploring different angles)
        eye_position_variance = abs(self.eye_servo - 25)  # How far from center
        if 10 < eye_position_variance < 40:
            reward += 0.5  # Reward for looking to sides
        
        # 2. STEERING REWARDS
        
        # Reward for smooth steering
        driver_change = abs(self.driver_servo - old_driver)
        if driver_change < 10:  # Smooth steering
            reward += 0.3
        
        # Reward for centering steering when path is clear
        if self.lidar_distance > 100 and abs(self.driver_servo - 90) < 10:
            reward += 0.5
        
        # 3. NAVIGATION REWARDS
        
        # Speed reward (encourage movement but not too fast)
        if 20 < self.motor_speed < 70:
            reward += 0.2
        
        # Distance reward (stay at safe distance)
        if self.lidar_distance > 80:
            reward += 1.0
        elif self.lidar_distance > 40:
            reward += 0.5
        elif self.lidar_distance < 20:
            reward -= 1.0  # Penalty for getting too close
        
        # 4. PROGRESS REWARD (encourage moving forward)
        if self.motor_speed > 30 and abs(self.driver_servo - 90) < 30:
            reward += 0.3
        
        return reward
    
    def render(self):
        """Simple text-based rendering"""
        print(f"\nStep {self.step_count:3d}:")
        print(f"  Eye Servo: {self.eye_servo:5.1f} | LiDAR: {self.lidar_distance:6.1f}cm")
        print(f"  Driver Servo: {self.driver_servo:5.1f}° | Speed: {self.motor_speed:4.0f}%")
        print(f"  Heading: {self.heading:5.1f}° | Pos: ({self.position[0]:.1f}, {self.position[1]:.1f})")

def train_spatial_ai():
    """Main training function"""
    print("="*70)
    print("SPATIAL AI TRAINING FOR TESLA AUTONOMOUS VEHICLE")
    print("="*70)
    print("Training the AI to understand:")
    print("  1. Eye servo position → LiDAR distance relationship")
    print("  2. How to steer (driver servo) based on what it 'sees'")
    print("  3. Speed control based on obstacle distances")
    print("="*70)
    
    # Initialize environment and agent
    env = SpatialTrainingEnv()
    agent = SpatialRLAgent()
    
    # Load existing model if available
    model_path = "AI/spatial_rl_model.pth"
    if os.path.exists(model_path):
        agent.load(model_path)
        print(f"✅ Loaded existing model from {model_path}")
    
    # Training parameters
    num_episodes = 500
    update_interval = 5  # Update every N steps
    save_interval = 50   # Save model every N episodes
    render_interval = 100
    
    print(f"\nStarting training for {num_episodes} episodes...")
    print(f"Model will be saved to: {model_path}")
    
    training_log = []
    
    for episode in range(num_episodes):
        # Reset environment
        state = env.reset()
        episode_reward = 0
        done = False
        
        # Determine if we should render this episode
        render = (episode % render_interval == 0) or (episode == num_episodes - 1)
        
        if render:
            print(f"\n{'='*60}")
            print(f"EPISODE {episode + 1}/{num_episodes}")
            print(f"{'='*60}")
        
        step = 0
        while not done:
            # Get action from agent
            action = agent.get_action(state, exploration_noise=0.3)
            
            # Take step in environment
            next_state, reward, done = env.step(action)
            
            # Store experience
            agent.store_experience(state, action, reward, next_state, done)
            
            # Update state
            state = next_state
            episode_reward += reward
            step += 1
            
            # Update agent periodically
            if step % update_interval == 0:
                losses = agent.update()
            
            # Render if needed
            if render and step % 20 == 0:
                env.render()
                print(f"  Action: EyeΔ={action[0]:+.2f}, DriverΔ={action[1]:+.2f}, Speed={action[2]:+.2f}")
                print(f"  Reward: {reward:+.2f} | Total: {episode_reward:+.2f}")
        
        # Store episode reward
        agent.episode_rewards.append(episode_reward)
        
        # Log training progress
        training_log.append({
            'episode': episode + 1,
            'reward': episode_reward,
            'steps': step,
            'avg_reward_10': np.mean(agent.episode_rewards[-10:]) if len(agent.episode_rewards) >= 10 else episode_reward
        })
        
        # Save model periodically
        if (episode + 1) % save_interval == 0:
            agent.save(model_path)
            print(f"💾 Saved model after episode {episode + 1}")
        
        # Print progress
        if not render and (episode + 1) % 10 == 0:
            avg_reward = np.mean(agent.episode_rewards[-10:])
            print(f"Episode {episode + 1:4d} | "
                  f"Reward: {episode_reward:7.2f} | "
                  f"Avg10: {avg_reward:7.2f} | "
                  f"Steps: {step:3d}")
    
    # Final save
    agent.save("AI/spatial_rl_model_final.pth")
    
    # Print training summary
    print(f"\n{'='*70}")
    print("TRAINING COMPLETE!")
    print(f"{'='*70}")
    
    final_rewards = agent.episode_rewards[-50:] if len(agent.episode_rewards) >= 50 else agent.episode_rewards
    if final_rewards:
        print(f"📊 Training Statistics:")
        print(f"   Total Episodes: {len(agent.episode_rewards)}")
        print(f"   Final Average Reward: {np.mean(final_rewards):.2f}")
        print(f"   Max Reward: {np.max(agent.episode_rewards):.2f}")
        print(f"   Min Reward: {np.min(agent.episode_rewards):.2f}")
        print(f"   Training Steps: {agent.training_step}")
    
    # Test the trained model
    print(f"\n🧪 TESTING TRAINED MODEL...")
    test_model(agent)
    
    return agent

def test_model(agent, num_tests=5):
    """Test the trained model"""
    env = SpatialTrainingEnv()
    
    test_rewards = []
    
    for test in range(num_tests):
        state = env.reset()
        total_reward = 0
        done = False
        
        print(f"\nTest {test + 1}/{num_tests}:")
        
        while not done:
            # Use deterministic actions (no exploration)
            action = agent.get_action(state, deterministic=True)
            state, reward, done = env.step(action)
            total_reward += reward
        
        test_rewards.append(total_reward)
        print(f"  Final Reward: {total_reward:.2f}")
        print(f"  Final Distance: {env.lidar_distance:.1f}cm")
        print(f"  Final Speed: {env.motor_speed:.0f}%")
    
    print(f"\n🎯 Test Results:")
    print(f"   Average Reward: {np.mean(test_rewards):.2f} ± {np.std(test_rewards):.2f}")
    print(f"   Best Test: {np.max(test_rewards):.2f}")
    
    return test_rewards

def export_for_vehicle(model_path="AI/spatial_rl_model_final.pth"):
    """Export trained model for use in vehicle"""
    if not os.path.exists(model_path):
        print(f"❌ Model not found: {model_path}")
        return
    
    # Create a simplified version for Raspberry Pi
    agent = SpatialRLAgent()
    agent.load(model_path)
    
    # Export just the actor network (for inference)
    export_data = {
        'actor_state_dict': agent.actor.state_dict(),
        'state_dim': agent.state_dim,
        'action_dim': agent.action_dim,
        'action_scales': {
            'eye_servo': 12.5,  # Multiply action[0] by this
            'driver_servo': 45,  # Multiply action[1] by this
            'motor_speed': 50    # (action[2] + 1) * 50
        }
    }
    
    export_path = "AI/vehicle_model.pth"
    torch.save(export_data, export_path)
    
    print(f"\n✅ Exported vehicle-ready model to: {export_path}")
    print(f"   State Dimension: {agent.state_dim}")
    print(f"   Action Dimension: {agent.action_dim}")
    print(f"   Model Size: {os.path.getsize(export_path) / 1024:.1f} KB")

if __name__ == "__main__":
    # Train the AI
    trained_agent = train_spatial_ai()
    
    # Export for vehicle use
    export_for_vehicle()
    
    print("\n" + "="*70)
    print("✅ TRAINING COMPLETE!")
    print("The AI now understands:")
    print("  • Eye servo position affects LiDAR readings")
    print("  • How to steer based on spatial awareness")
    print("  • Speed control based on obstacle distances")
    print("\nNext steps:")
    print("  1. Copy AI/vehicle_model.pth to your vehicle")
    print("  2. Use the TeslaSpatialAIController (below)")
    print("  3. The car will now actively scan and understand space!")
    print("="*70)