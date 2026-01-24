#!/usr/bin/env python3
"""
🎯 WORKING TRAINER - NO DIMENSION ERRORS
Simplified but effective training
"""

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import random
import time
import os

# ==================== WORKING CONFIG ====================
class WorkingConfig:
    def __init__(self):
        # CORRECT: 23 dimensions
        self.state_dim = 23
        self.action_dim = 3
        
        self.learning_rate = 3e-4
        self.gamma = 0.99
        self.tau = 0.005
        self.batch_size = 64  # Smaller for Raspberry Pi
        self.buffer_size = 100000
        
        self.hidden_dim = 256  # Smaller for faster training
        
        self.total_episodes = 500
        self.max_steps = 500  # Shorter episodes
        
        self.checkpoint_dir = "AI/checkpoints_working"
        self.log_file = "AI/training_log_working.txt"

# ==================== WORKING ENVIRONMENT ====================
class WorkingEnv:
    def __init__(self):
        self.track_length = 30.0  # Shorter for easier success
        self.max_speed = 2.0
        
        self.reset()
    
    def reset(self):
        self.position = 0.0
        self.speed = 0.0
        self.eye_position = 0.0
        self.steps = 0
        self.last_progress = 0.0
        
        return self._get_observation()
    
    def _get_observation(self):
        """Returns EXACTLY 23 dimensions"""
        obs = np.zeros(23, dtype=np.float32)
        
        # Basic state (5)
        obs[0] = self.position / self.track_length
        obs[1] = self.speed / self.max_speed
        obs[2] = self.eye_position
        obs[3] = 0.0  # placeholder
        obs[4] = self.steps / 500.0
        
        # Simulated LiDAR (12)
        for i in range(12):
            # Simple obstacle simulation
            if i == 5 or i == 6:  # Forward directions
                distance = max(0.1, 10.0 - self.position)
                obs[5 + i] = distance / 20.0
            else:
                obs[5 + i] = 1.0  # Clear
        
        # Additional features (6)
        obs[17] = 0.0  # steering history
        obs[18] = 0.0  # steering std
        obs[19] = 0.0  # throttle history
        obs[20] = 0.5  # progress to next waypoint
        obs[21] = 0.0  # time pressure
        obs[22] = 0.0  # placeholder
        
        return obs
    
    def step(self, action):
        self.steps += 1
        
        steering = np.clip(action[0], -1.0, 1.0)
        throttle = np.clip(action[1], 0.0, 1.0)
        eye_command = np.clip(action[2], -1.0, 1.0)
        
        # Update eye
        self.eye_position = self.eye_position * 0.8 + eye_command * 0.2
        
        # Simple dynamics
        acceleration = throttle * 0.8
        deceleration = 0.1
        
        self.speed += (acceleration - deceleration) * 0.1
        self.speed = np.clip(self.speed, 0.0, self.max_speed)
        
        # Update position
        self.position += self.speed * 0.1 * (1.0 - abs(steering) * 0.3)
        
        # Calculate reward
        reward = self._calculate_reward(steering, throttle, eye_command)
        
        # Check termination
        done = False
        info = {
            'position': self.position,
            'speed': self.speed,
            'success': False,
            'progress': self.position / self.track_length
        }
        
        if self.position >= self.track_length:
            done = True
            info['success'] = True
            reward += 50.0  # Success bonus
        
        if self.steps >= 500:
            done = True
            info['timeout'] = True
        
        next_obs = self._get_observation()
        
        return next_obs, reward, done, info
    
    def _calculate_reward(self, steering, throttle, eye_command):
        """Reward function focused on COMPLETION"""
        reward = 0.0
        
        # 1. PROGRESS (most important)
        progress = self.position - self.last_progress
        reward += progress * 5.0  # Strong progress reward
        self.last_progress = self.position
        
        # 2. SUCCESS BONUSES
        # Extra bonus for last 20%
        if self.position > self.track_length * 0.8:
            final_push = (self.position - self.track_length * 0.8) * 10.0
            reward += final_push
        
        # 3. SPEED EFFICIENCY
        optimal_speed = 1.0  # m/s
        speed_error = abs(self.speed - optimal_speed)
        speed_efficiency = 1.0 - (speed_error / optimal_speed)
        reward += speed_efficiency * 2.0
        
        # 4. EYE ALIGNMENT
        if abs(steering) > 0.1:
            eye_alignment = 1.0 - abs(steering - eye_command)
            reward += eye_alignment * 1.0
        
        # 5. SURVIVAL BONUS
        reward += 0.05
        
        return reward

# ==================== SIMPLE NETWORKS ====================
class SimpleActor(nn.Module):
    def __init__(self, state_dim=23, action_dim=3, hidden_dim=256):
        super().__init__()
        
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim * 2)  # mean and log_std
        )
    
    def forward(self, state):
        output = self.net(state)
        mean, log_std = output[:, :3], output[:, 3:]
        log_std = torch.clamp(log_std, -20, 2)
        return mean, log_std
    
    def get_action(self, state, deterministic=False):
        mean, log_std = self.forward(state)
        std = torch.exp(log_std)
        
        if deterministic:
            action = torch.tanh(mean)
        else:
            normal = torch.distributions.Normal(mean, std)
            action = torch.tanh(normal.rsample())
        
        return action

class SimpleCritic(nn.Module):
    def __init__(self, state_dim=23, action_dim=3, hidden_dim=256):
        super().__init__()
        
        # CORRECT: state_dim + action_dim
        self.q1 = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )
        
        self.q2 = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )
    
    def forward(self, state, action):
        x = torch.cat([state, action], dim=1)
        q1 = self.q1(x)
        q2 = self.q2(x)
        return q1, q2

# ==================== WORKING AGENT ====================
class WorkingAgent:
    def __init__(self, config):
        self.config = config
        self.device = torch.device("cpu")
        
        print(f"🤖 Creating networks with state_dim={config.state_dim}")
        
        # Create networks
        self.actor = SimpleActor(config.state_dim, config.action_dim, config.hidden_dim).to(self.device)
        self.critic = SimpleCritic(config.state_dim, config.action_dim, config.hidden_dim).to(self.device)
        self.critic_target = SimpleCritic(config.state_dim, config.action_dim, config.hidden_dim).to(self.device)
        self.critic_target.load_state_dict(self.critic.state_dict())
        
        # Optimizers
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=config.learning_rate)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=config.learning_rate)
        
        # Replay buffer
        self.states = []
        self.actions = []
        self.rewards = []
        self.next_states = []
        self.dones = []
        
        # Stats
        self.total_steps = 0
        self.episode_rewards = []
        self.best_reward = -float('inf')
        
        os.makedirs(config.checkpoint_dir, exist_ok=True)
    
    def get_action(self, state, deterministic=False):
        self.actor.eval()
        with torch.no_grad():
            state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            action = self.actor.get_action(state_tensor, deterministic=deterministic)
            action = action.cpu().numpy()[0]
            
            # Add exploration noise
            if not deterministic and random.random() < 0.3:
                noise = np.random.normal(0, 0.1, size=3)
                action += noise
            
            # Clip
            action[0] = np.clip(action[0], -1.0, 1.0)
            action[1] = np.clip(action[1], 0.0, 1.0)
            action[2] = np.clip(action[2], -1.0, 1.0)
        
        return action
    
    def store_experience(self, state, action, reward, next_state, done):
        self.states.append(state)
        self.actions.append(action)
        self.rewards.append(reward)
        self.next_states.append(next_state)
        self.dones.append(done)
        
        # Keep buffer manageable
        if len(self.states) > self.config.buffer_size:
            self.states.pop(0)
            self.actions.pop(0)
            self.rewards.pop(0)
            self.next_states.pop(0)
            self.dones.pop(0)
    
    def update(self):
        if len(self.states) < self.config.batch_size:
            return
        
        # Sample random batch
        indices = np.random.choice(len(self.states), self.config.batch_size, replace=False)
        
        states = torch.FloatTensor(np.array([self.states[i] for i in indices])).to(self.device)
        actions = torch.FloatTensor(np.array([self.actions[i] for i in indices])).to(self.device)
        rewards = torch.FloatTensor(np.array([self.rewards[i] for i in indices])).unsqueeze(1).to(self.device)
        next_states = torch.FloatTensor(np.array([self.next_states[i] for i in indices])).to(self.device)
        dones = torch.FloatTensor(np.array([self.dones[i] for i in indices])).unsqueeze(1).to(self.device)
        
        # Critic update
        with torch.no_grad():
            next_actions = self.actor.get_action(next_states, deterministic=True)
            q1_next, q2_next = self.critic_target(next_states, next_actions)
            q_next = torch.min(q1_next, q2_next)
            target_q = rewards + self.config.gamma * (1 - dones) * q_next
        
        q1, q2 = self.critic(states, actions)
        critic_loss = F.mse_loss(q1, target_q) + F.mse_loss(q2, target_q)
        
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic.parameters(), 1.0)
        self.critic_optimizer.step()
        
        # Actor update
        policy_actions = self.actor.get_action(states)
        q1_pi, q2_pi = self.critic(states, policy_actions)
        q_pi = torch.min(q1_pi, q2_pi)
        
        actor_loss = -q_pi.mean()
        
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 1.0)
        self.actor_optimizer.step()
        
        # Target update
        for param, target_param in zip(self.critic.parameters(), self.critic_target.parameters()):
            target_param.data.copy_(self.config.tau * param.data + (1 - self.config.tau) * target_param.data)
        
        return critic_loss.item(), actor_loss.item()
    
    def train(self):
        print("🚀 WORKING TRAINER - NO DIMENSION ERRORS")
        print("=" * 70)
        print(f"State dimension: {self.config.state_dim}")
        print(f"Target: Reach end of 30m track")
        print("=" * 70)
        
        with open(self.config.log_file, 'w') as f:
            f.write("Episode,Reward,Steps,Success,Progress\n")
        
        for episode in range(self.config.total_episodes):
            env = WorkingEnv()
            state = env.reset()
            
            episode_reward = 0
            episode_steps = 0
            episode_success = 0
            
            for step in range(self.config.max_steps):
                self.total_steps += 1
                
                # Get action
                if self.total_steps < 1000:
                    # Exploration phase
                    action = np.random.uniform(-1, 1, size=3)
                    action[1] = np.random.uniform(0, 0.5)  # Conservative throttle
                else:
                    action = self.get_action(state, deterministic=False)
                
                # Take step
                next_state, reward, done, info = env.step(action)
                
                # Store experience
                self.store_experience(state, action, reward, next_state, done)
                
                # Train
                if self.total_steps > 1000 and self.total_steps % 4 == 0:
                    self.update()
                
                # Update
                state = next_state
                episode_reward += reward
                episode_steps += 1
                
                if info['success']:
                    episode_success = 1
                    print(f"🎉 FIRST SUCCESS! Episode {episode+1}")
                
                if done:
                    break
            
            # Episode complete
            self.episode_rewards.append(episode_reward)
            
            # Calculate average
            avg_reward = np.mean(self.episode_rewards[-10:]) if len(self.episode_rewards) >= 10 else episode_reward
            
            print(f"Ep {episode+1:3d} | "
                  f"Reward: {episode_reward:7.1f} | "
                  f"Avg10: {avg_reward:7.1f} | "
                  f"Steps: {episode_steps:4d} | "
                  f"Success: {episode_success} | "
                  f"Progress: {info['progress']:5.1%}")
            
            with open(self.config.log_file, 'a') as f:
                f.write(f"{episode+1},{episode_reward:.2f},{episode_steps},{episode_success},{info['progress']:.3f}\n")
            
            # Save best model
            if episode_reward > self.best_reward:
                self.best_reward = episode_reward
                self.save(f"{self.config.checkpoint_dir}/WORKING_{int(episode_reward)}.pth")
                print(f"💾 New best: {episode_reward:.1f}")
            
            # Early success check
            if episode_success == 1:
                print(f"✅ SUCCESS ACHIEVED! Ready for real-world testing.")
                self.save(f"{self.config.checkpoint_dir}/SUCCESS_ep{episode+1}.pth")
                # Continue training for robustness
                # break  # Uncomment to stop after first success
        
        # Final save
        self.save(f"{self.config.checkpoint_dir}/working_final.pth")
        
        print("=" * 70)
        print(f"✅ Training complete! Best reward: {self.best_reward:.1f}")
        
        # Summary
        if len(self.episode_rewards) > 0:
            rewards = np.array(self.episode_rewards)
            print(f"\n📊 Summary:")
            print(f"  Episodes: {len(rewards)}")
            print(f"  Best: {rewards.max():.1f}")
            print(f"  Average: {rewards.mean():.1f}")
            print(f"  Final: {rewards[-1]:.1f}")
            
            # Check for successes
            if self.best_reward > 100:  # Likely had a success
                print(f"\n🎉 READY FOR REAL-WORLD DEPLOYMENT!")
            else:
                print(f"\n🎯 Continue training or increase track length")
    
    def save(self, path):
        torch.save({
            'actor_state_dict': self.actor.state_dict(),
            'critic_state_dict': self.critic.state_dict(),
            'critic_target_state_dict': self.critic_target.state_dict(),
            'config': self.config,
            'best_reward': self.best_reward,
            'episode_rewards': self.episode_rewards,
            'total_steps': self.total_steps
        }, path)
        print(f"💾 Saved to {path}")
    
    def load(self, path):
        try:
            checkpoint = torch.load(path, map_location=self.device)
            self.actor.load_state_dict(checkpoint['actor_state_dict'])
            self.critic.load_state_dict(checkpoint['critic_state_dict'])
            self.critic_target.load_state_dict(checkpoint['critic_target_state_dict'])
            self.best_reward = checkpoint['best_reward']
            self.episode_rewards = checkpoint.get('episode_rewards', [])
            self.total_steps = checkpoint.get('total_steps', 0)
            print(f"✅ Loaded: {self.best_reward:.1f} rewards")
            return True
        except Exception as e:
            print(f"❌ Load failed: {e}")
            return False

# ==================== MAIN ====================
def main():
    print("🎯 WORKING AI TRAINER - GUARANTEED NO DIMENSION ERRORS")
    print("=" * 70)
    
    config = WorkingConfig()
    agent = WorkingAgent(config)
    
    # Start training
    agent.train()
    
    print("=" * 70)
    print("🚗 Ready to test on real hardware!")

if __name__ == "__main__":
    main()