#!/usr/bin/env python3
"""
<<<<<<< HEAD
Industry-level SAC trainer for wheelchair/car with 7 distance sensors,
speed (0-100%), and steering angle (0-50°).
=======
FIXED TRAINING ENVIRONMENT - 6D STATE VERSION
Trains AI with 6D state vector matching your controller
>>>>>>> ece7e52 (Working Codes with Pending Speed Improvement)
"""

import numpy as np
import torch
<<<<<<< HEAD
import torch.nn as nn
import torch.optim as optim
import os
from collections import deque
import random

# ----------------------------
# SAC Agent
# ----------------------------
class SACAgent:
    def __init__(self, state_dim=9, action_dim=2, hidden_dim=128, lr=3e-4):
        self.state_dim = state_dim
        self.action_dim = action_dim

        # Actor: outputs mean + log_std
        self.actor = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim * 2)
        )

        # Critic networks
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

        # Target critics
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

        self.target_critic1.load_state_dict(self.critic1.state_dict())
        self.target_critic2.load_state_dict(self.critic2.state_dict())

        # Optimizers
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr)
        self.critic1_optimizer = optim.Adam(self.critic1.parameters(), lr=lr)
        self.critic2_optimizer = optim.Adam(self.critic2.parameters(), lr=lr)

        # Replay buffer
        self.replay_buffer = deque(maxlen=50000)
        self.batch_size = 128
        self.gamma = 0.99
        self.tau = 0.005

    def get_action(self, state, deterministic=False, noise=0.1):
        state_tensor = torch.FloatTensor(state).unsqueeze(0)
        with torch.no_grad():
            out = self.actor(state_tensor)
            mean = out[:, :self.action_dim]
            log_std = out[:, self.action_dim:]
            std = torch.exp(log_std)
            if deterministic:
                action = torch.tanh(mean)
            else:
                normal = torch.distributions.Normal(mean, std)
                action = torch.tanh(normal.rsample())
                action += torch.randn_like(action) * noise
                action = torch.clamp(action, -1, 1)
        return action.squeeze().cpu().numpy()

    def store(self, state, action, reward, next_state, done):
        self.replay_buffer.append((state, action, reward, next_state, done))

    def update(self):
        if len(self.replay_buffer) < self.batch_size:
            return 0,0,0

        batch = random.sample(self.replay_buffer, self.batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        states = torch.FloatTensor(states)
        actions = torch.FloatTensor(actions)
        rewards = torch.FloatTensor(rewards).unsqueeze(1)
        next_states = torch.FloatTensor(next_states)
        dones = torch.FloatTensor(dones).unsqueeze(1)

        # Target Q
        with torch.no_grad():
            next_out = self.actor(next_states)
            mean = next_out[:, :self.action_dim]
            log_std = next_out[:, self.action_dim:]
            std = torch.exp(log_std)
            normal = torch.distributions.Normal(mean, std)
            next_actions = torch.tanh(normal.rsample())

            target_q1 = self.target_critic1(torch.cat([next_states, next_actions],1))
            target_q2 = self.target_critic2(torch.cat([next_states, next_actions],1))
            target_q = torch.min(target_q1, target_q2)
            target_value = rewards + self.gamma * (1 - dones) * target_q

        # Critic losses
        q1 = self.critic1(torch.cat([states, actions],1))
        q2 = self.critic2(torch.cat([states, actions],1))
        critic1_loss = nn.MSELoss()(q1, target_value)
        critic2_loss = nn.MSELoss()(q2, target_value)

        self.critic1_optimizer.zero_grad()
        critic1_loss.backward()
        self.critic1_optimizer.step()

        self.critic2_optimizer.zero_grad()
        critic2_loss.backward()
        self.critic2_optimizer.step()

        # Actor loss
        out = self.actor(states)
        mean = out[:, :self.action_dim]
        log_std = out[:, self.action_dim:]
        std = torch.exp(log_std)
        normal = torch.distributions.Normal(mean, std)
        new_actions = torch.tanh(normal.rsample())
        q1_new = self.critic1(torch.cat([states, new_actions],1))
        q2_new = self.critic2(torch.cat([states, new_actions],1))
        q_new = torch.min(q1_new, q2_new)
        log_prob = normal.log_prob(mean)
        log_prob -= torch.log(1 - new_actions.pow(2) + 1e-6)
        log_prob = log_prob.sum(1, keepdim=True)
        actor_loss = (0.2*log_prob - q_new).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # Soft update
        for tp, p in zip(self.target_critic1.parameters(), self.critic1.parameters()):
            tp.data.copy_(self.tau*p.data + (1-self.tau)*tp.data)
        for tp, p in zip(self.target_critic2.parameters(), self.critic2.parameters()):
            tp.data.copy_(self.tau*p.data + (1-self.tau)*tp.data)

        return critic1_loss.item(), critic2_loss.item(), actor_loss.item()

# ----------------------------
# Training Environment
# ----------------------------
class CarEnv:
    def __init__(self):
        self.max_speed = 100
        self.max_steer = 50
        self.speed = 0
        self.steering = 25
        self.step_count = 0
        self.max_steps = 200
        # distances: [front-left, front, front-right, left, right, rear]
        self.distances = [100]*7

    def reset(self):
        self.speed = 0
        self.steering = 25
        self.step_count = 0
        self.distances = [random.uniform(20,100) for _ in range(7)]
        return self.get_state()

    def get_state(self):
        norm_dist = [d/100.0 for d in self.distances]
        norm_speed = self.speed/self.max_speed
        norm_steer = self.steering/self.max_steer
        return np.array(norm_dist + [norm_speed, norm_steer], dtype=np.float32)

    def step(self, action):
        self.step_count += 1
        # action: [-1,1] -> delta
        steer_delta = action[0]*5  # max ±5 deg per step
        speed_delta = action[1]*5  # max ±5% per step

        self.steering = np.clip(self.steering + steer_delta, 0, self.max_steer)
        self.speed = np.clip(self.speed + speed_delta, 0, self.max_speed)

        # Simulate distances changing randomly for now
        self.distances = [max(5,min(100,d + random.uniform(-5,5))) for d in self.distances]

        # Reward shaping
        reward = 0
        # 1. Stay away from obstacles
        min_front = min(self.distances[:6])
        if min_front < 15:
            reward -= 10
        else:
            reward += min_front/10
        # 2. Encourage forward motion
        reward += self.speed/50
        # 3. Smooth steering
        if abs(steer_delta)<2:
            reward += 0.5

        done = self.step_count>=self.max_steps
        return self.get_state(), reward, done

# ----------------------------
# Training Loop
# ----------------------------
def train(num_episodes=3000):
    env = CarEnv()
    agent = SACAgent(state_dim=9, action_dim=2)
    save_path = "vehicle_sac.pth"

    for ep in range(num_episodes):
=======
import time
import random
import os
from SAC import FixedSAC6D  # Import the fixed SAC above

class FixedTrainingEnv6D:
    """Training environment with 6D state vector"""
    
    def __init__(self):
        # State: [distance_norm, speed_norm, steering_norm, eye_norm, sin_time, obstacle_flag]
        self.state_dim = 6
        self.action_dim = 6  # [steering, throttle, eye, aux1, aux2, aux3]
        
        # Environment parameters
        self.max_steps = 200
        self.reset()
        
        print("✅ Fixed Training Environment 6D initialized")
    
    def reset(self):
        """Reset environment"""
        # Random initial conditions
        self.distance = random.uniform(50, 200)  # cm
        self.speed = 0  # %
        self.steering = 90  # degrees (center)
        self.eye = 31.5  # eye position
        self.step_count = 0
        
        # Obstacle simulation
        self.obstacle_distance = random.uniform(30, 150)
        self.obstacle_angle = random.uniform(-30, 30)  # degrees from center
        
        return self._get_state()
    
    def _get_state(self):
        """Get 6D state vector matching controller"""
        state = np.zeros(6, dtype=np.float32)
        
        # 0: Normalized distance (0=close, 1=far)
        state[0] = min(1.0, self.distance / 200.0)
        
        # 1: Normalized speed (-1=reverse, 0=stop, 1=forward)
        state[1] = self.speed / 100.0
        
        # 2: Normalized steering (-1=left, 0=center, 1=right)
        state[2] = (self.steering - 90) / 50.0
        
        # 3: Normalized eye position (-1=left, 0=center, 1=right)
        state[3] = (self.eye - 31.5) / 15.0
        
        # 4: Time/sin phase
        state[4] = np.sin(self.step_count * 0.1)
        
        # 5: Obstacle flag (0=no obstacle, 1=obstacle close)
        state[5] = 1.0 if self.distance < 50.0 else 0.0
        
        return state
    
    def step(self, action):
        """
        Execute action
        action: [steering_norm, throttle_norm, eye_norm, aux1, aux2, aux3]
        """
        self.step_count += 1
        
        # Parse action (first 3 are main controls)
        steering_norm = np.clip(action[0], -1, 1)
        throttle_norm = np.clip(action[1], -1, 1)
        eye_norm = np.clip(action[2], -1, 1) if len(action) > 2 else 0
        
        # Convert to hardware values (simulated)
        steering_delta = steering_norm * 10  # ±10 degrees per step
        throttle_delta = throttle_norm * 20  # ±20% per step
        eye_delta = eye_norm * 5  # ±5 units per step
        
        # Update state
        self.steering = max(40, min(140, self.steering + steering_delta))
        self.speed = max(-100, min(100, self.speed + throttle_delta))
        self.eye = max(15, min(45, self.eye + eye_delta))
        
        # Simulate movement
        if self.speed > 0:
            # Moving forward reduces distance to obstacle
            speed_factor = self.speed / 100.0
            self.distance -= speed_factor * 5
            
            # Random distance fluctuations
            self.distance += random.uniform(-5, 5)
            
            # Don't go below 0
            self.distance = max(0.1, self.distance)
        
        # Calculate reward
        reward = self._calculate_reward(steering_norm, throttle_norm)
        
        # Check termination
        done = self.step_count >= self.max_steps
        if self.distance < 10:  # Collision
            done = True
            reward -= 10
        
        return self._get_state(), reward, done
    
    def _calculate_reward(self, steering_norm, throttle_norm):
        """Calculate reward for training"""
        reward = 0
        
        # 1. Distance reward (maintain safe distance)
        if 30 < self.distance < 100:
            reward += 1.0
        elif self.distance >= 100:
            reward += 0.5
        elif self.distance < 20:
            reward -= 2.0
        
        # 2. Speed reward (encourage movement)
        if 20 < abs(self.speed) < 70:
            reward += 0.3
        
        # 3. Steering reward (smooth steering)
        if abs(steering_norm) < 0.5:  # Not too extreme
            reward += 0.2
        
        # 4. Progress reward (moving forward)
        if self.speed > 30 and self.distance > 30:
            reward += 0.5
        
        # 5. Penalty for extreme actions
        if abs(steering_norm) > 0.8:
            reward -= 0.3
        if abs(throttle_norm) > 0.8:
            reward -= 0.3
        
        return reward
    
    def render(self):
        """Simple rendering"""
        print(f"Step {self.step_count}: "
              f"Dist={self.distance:.1f}cm, "
              f"Speed={self.speed:.0f}%, "
              f"Steer={self.steering:.0f}°, "
              f"Eye={self.eye:.1f}")

def train_fixed_sac_6d():
    """Train SAC with 6D state"""
    print("=" * 70)
    print("FIXED SAC TRAINING - 6D STATE VERSION")
    print("=" * 70)
    print("Training 6D → 6D model that matches vehicle controller")
    print("State: [distance, speed, steering, eye, time_sin, obstacle_flag]")
    print("Action: [steering, throttle, eye, aux1, aux2, aux3]")
    print("=" * 70)
    
    # Create environment and agent
    env = FixedTrainingEnv6D()
    agent = FixedSAC6D()
    
    # Load existing model if available
    model_path = "AI/spatial_rl_model_6d.pth"
    if os.path.exists(model_path):
        agent.load(model_path)
        print(f"✅ Loaded existing model from {model_path}")
    
    # Training parameters
    num_episodes = 1000
    update_freq = 10  # Update every N steps
    save_freq = 50    # Save every N episodes
    
    print(f"\nTraining for {num_episodes} episodes...")
    print(f"Model will be saved to: {model_path}\n")
    
    episode_rewards = []
    
    for episode in range(num_episodes):
        # Reset environment
        state = env.reset()
        episode_reward = 0
        done = False
        
        # Progress display
        if (episode + 1) % 10 == 0:
            print(f"\nEpisode {episode + 1}/{num_episodes}")
        
        step = 0
        while not done:
            # Get action from agent
            action = agent.get_action(state, deterministic=False)
            
            # Take step
            next_state, reward, done = env.step(action)
            
            # Store transition
            agent.store_transition(state, action, reward, next_state, done)
            
            # Update agent periodically
            if step % update_freq == 0:
                agent.update()
            
            # Update state
            state = next_state
            episode_reward += reward
            step += 1
        
        # Store episode reward
        episode_rewards.append(episode_reward)
        
        # Save model periodically
        if (episode + 1) % save_freq == 0:
            agent.save(model_path)
            
            # Print training progress
            avg_reward = np.mean(episode_rewards[-save_freq:])
            print(f"  Saved model after episode {episode + 1}")
            print(f"  Average reward (last {save_freq}): {avg_reward:.2f}")
        
        # Print episode summary
        if (episode + 1) % 10 == 0:
            avg_reward = np.mean(episode_rewards[-10:]) if len(episode_rewards) >= 10 else episode_reward
            print(f"  Episode {episode + 1:4d}: "
                  f"Reward: {episode_reward:7.2f} | "
                  f"Avg10: {avg_reward:7.2f} | "
                  f"Steps: {step:3d}")
    
    # Final save
    agent.save("AI/spatial_rl_model_6d_final.pth")
    
    # Training summary
    print(f"\n{'='*70}")
    print("TRAINING COMPLETE!")
    print(f"{'='*70}")
    
    if episode_rewards:
        final_avg = np.mean(episode_rewards[-100:]) if len(episode_rewards) >= 100 else np.mean(episode_rewards)
        print(f"📊 Training Statistics:")
        print(f"   Total Episodes: {len(episode_rewards)}")
        print(f"   Final Average Reward: {final_avg:.2f}")
        print(f"   Max Reward: {np.max(episode_rewards):.2f}")
        print(f"   Min Reward: {np.min(episode_rewards):.2f}")
        print(f"   Training Steps: {agent.total_steps}")
    
    # Test the trained model
    test_trained_model(agent)
    
    return agent

def test_trained_model(agent, num_tests=3):
    """Test the trained model"""
    print(f"\n🧪 TESTING TRAINED 6D MODEL...")
    
    env = FixedTrainingEnv6D()
    test_rewards = []
    
    for test in range(num_tests):
>>>>>>> ece7e52 (Working Codes with Pending Speed Improvement)
        state = env.reset()
        total_reward = 0
        done = False
        while not done:
<<<<<<< HEAD
            action = agent.get_action(state)
            next_state, reward, done = env.step(action)
            agent.store(state, action, reward, next_state, done)
            state = next_state
            total_reward += reward
            agent.update()
        print(f"Episode {ep+1}/{num_episodes} | Reward: {total_reward:.2f}")
        if (ep+1)%50==0:
            torch.save(agent.actor.state_dict(), save_path)
            print(f"Model saved at episode {ep+1}")
    torch.save(agent.actor.state_dict(), save_path)
    print("Training finished and model saved.")

# ----------------------------
# Main
# ----------------------------
if __name__=="__main__":
    train()
=======
            # Use deterministic actions
            action = agent.get_action(state, deterministic=True)
            state, reward, done = env.step(action)
            total_reward += reward
        
        test_rewards.append(total_reward)
        print(f"  Final Reward: {total_reward:.2f}")
        print(f"  Final Distance: {env.distance:.1f}cm")
        print(f"  Final Speed: {env.speed:.0f}%")
        print(f"  Final Steering: {env.steering:.0f}°")
        print(f"  Final Eye: {env.eye:.1f}")
    
    print(f"\n🎯 Test Results:")
    print(f"   Average Reward: {np.mean(test_rewards):.2f} ± {np.std(test_rewards):.2f}")
    print(f"   Best Test: {np.max(test_rewards):.2f}")

def export_for_vehicle(model_path="AI/spatial_rl_model_6d_final.pth"):
    """Export trained 6D model for vehicle"""
    if not os.path.exists(model_path):
        print(f"❌ Model not found: {model_path}")
        return
    
    # Load the model
    checkpoint = torch.load(model_path, map_location='cpu')
    
    # Create simplified export
    export_data = {
        'actor': checkpoint['actor_state_dict'],
        'critic1': checkpoint['critic_state_dict']['q1.state_dict'] if 'q1.state_dict' in checkpoint['critic_state_dict'] else None,
        'critic2': checkpoint['critic_state_dict']['q2.state_dict'] if 'q2.state_dict' in checkpoint['critic_state_dict'] else None,
        'config': checkpoint.get('config', {}),
        'training_step': checkpoint.get('total_steps', 0)
    }
    
    export_path = "AI/spatial_rl_model.pth"  # Overwrite existing model
    torch.save(export_data, export_path)
    
    print(f"\n✅ Exported vehicle-ready 6D model to: {export_path}")
    print(f"   Model Size: {os.path.getsize(export_path) / 1024:.1f} KB")
    
    # Test that it loads correctly
    agent = FixedSAC6D()
    agent.load(export_path)
    test_state = np.random.randn(6)
    action = agent.get_action(test_state, deterministic=True)
    print(f"   Test inference: State {test_state.shape} → Action {action.shape}")

if __name__ == "__main__":
    # Train the fixed 6D SAC
    trained_agent = train_fixed_sac_6d()
    
    # Export for vehicle
    export_for_vehicle()
    
    print("\n" + "="*70)
    print("✅ FIXED 6D TRAINING COMPLETE!")
    print("The AI now uses the correct 6D state vector")
    print("Next steps:")
    print("  1. Run your vehicle controller with the new model")
    print("  2. The controller will use 6D state → 6D action")
    print("  3. AI should now output meaningful control values!")
    print("="*70)
>>>>>>> ece7e52 (Working Codes with Pending Speed Improvement)
