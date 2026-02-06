#!/usr/bin/env python3
"""
FIXED SAC (Soft Actor-Critic) - 6D STATE VERSION
Matches your vehicle controller: 6D state → 6D action
"""

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Normal
import random
from collections import deque
import os

class FixedSACActor6D(nn.Module):
    """Actor network: 6 → 128 → 128 → 6 (EXACT match)"""
    
    def __init__(self, log_std_min=-20, log_std_max=2):
        super().__init__()
        
        self.log_std_min = log_std_min
        self.log_std_max = log_std_max
        
        # EXACT architecture from your model
        # Layer 0: 6 → 128
        self.fc0 = nn.Linear(6, 128)
        
        # Layer 2: 128 → 128
        self.fc2 = nn.Linear(128, 128)
        
        # Layer 4: 128 → 6 (mean) + 6 (log_std)
        self.fc4_mean = nn.Linear(128, 6)
        self.fc4_log_std = nn.Linear(128, 6)
        
        # Initialize weights
        self.apply(self._init_weights)
    
    def _init_weights(self, m):
        if isinstance(m, nn.Linear):
            nn.init.orthogonal_(m.weight, gain=np.sqrt(2))
            nn.init.constant_(m.bias, 0.0)
    
    def forward(self, state, deterministic=False, with_logprob=True):
        """Forward pass matching your model file"""
        # Layer 0
        x = F.relu(self.fc0(state))
        
        # Layer 2
        x = F.relu(self.fc2(x))
        
        # Layer 4: mean and log_std
        mean = self.fc4_mean(x)
        log_std = self.fc4_log_std(x)
        log_std = torch.clamp(log_std, self.log_std_min, self.log_std_max)
        std = torch.exp(log_std)
        
        # Gaussian distribution
        normal = Normal(mean, std)
        
        if deterministic:
            # Deterministic action for evaluation
            action = torch.tanh(mean)
            if with_logprob:
                return action, None
            return action
        else:
            # Sample from distribution
            x_t = normal.rsample()
            action = torch.tanh(x_t)
            
            if with_logprob:
                # Compute log probability with correction for tanh
                log_prob = normal.log_prob(x_t)
                log_prob -= torch.log(1 - action.pow(2) + 1e-6)
                log_prob = log_prob.sum(1, keepdim=True)
                return action, log_prob
            
            return action

class FixedSACCritic6D(nn.Module):
    """Critic network: state(6) + action(6) → 128 → 128 → 1"""
    
    def __init__(self):
        super().__init__()
        
        self.q1 = nn.Sequential(
            nn.Linear(6 + 6, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, 1)
        )
        
        self.q2 = nn.Sequential(
            nn.Linear(6 + 6, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, 1)
        )
    
    def forward(self, state, action):
        """Forward pass through both critics"""
        x = torch.cat([state, action], dim=1)
        return self.q1(x), self.q2(x)

class FixedSAC6D:
    """Complete SAC agent for 6D state"""
    
    def __init__(self, 
                 gamma=0.99,
                 tau=0.005,
                 alpha=0.2,
                 lr=3e-4,
                 hidden_dim=128,
                 batch_size=256,
                 buffer_size=100000):
        
        # Hyperparameters
        self.gamma = gamma
        self.tau = tau
        self.alpha = alpha
        self.batch_size = batch_size
        
        # Device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Using device: {self.device}")
        
        # Networks
        self.actor = FixedSACActor6D().to(self.device)
        self.critic = FixedSACCritic6D().to(self.device)
        self.critic_target = FixedSACCritic6D().to(self.device)
        
        # Copy weights to target
        self.critic_target.load_state_dict(self.critic.state_dict())
        
        # Optimizers
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=lr)
        
        # Automatic temperature tuning
        self.target_entropy = -6  # -action_dim
        self.log_alpha = torch.tensor(np.log(alpha), requires_grad=True, device=self.device)
        self.alpha_optimizer = optim.Adam([self.log_alpha], lr=lr)
        
        # Replay buffer
        self.replay_buffer = deque(maxlen=buffer_size)
        
        # Training stats
        self.total_steps = 0
        
        print("✅ Fixed SAC 6D initialized: 6D state → 6D action")
    
    def get_action(self, state, deterministic=False):
        """Get action for given state"""
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        
        with torch.no_grad():
            if deterministic:
                action = self.actor(state_tensor, deterministic=True, with_logprob=False)
            else:
                action, _ = self.actor(state_tensor, deterministic=False, with_logprob=True)
        
        action = action.cpu().numpy().squeeze(0)
        
        # Add exploration noise
        if not deterministic:
            noise_scale = 0.1  # Exploration noise
            noise = np.random.normal(0, noise_scale, size=6)
            action = np.clip(action + noise, -1, 1)
        
        return action
    
    def store_transition(self, state, action, reward, next_state, done):
        """Store transition in replay buffer"""
        self.replay_buffer.append((state, action, reward, next_state, done))
    
    def update(self):
        """Update networks"""
        if len(self.replay_buffer) < self.batch_size:
            return {}
        
        # Sample batch
        batch = random.sample(self.replay_buffer, self.batch_size)
        
        # Unpack batch
        states = torch.FloatTensor(np.array([e[0] for e in batch])).to(self.device)
        actions = torch.FloatTensor(np.array([e[1] for e in batch])).to(self.device)
        rewards = torch.FloatTensor(np.array([e[2] for e in batch])).unsqueeze(1).to(self.device)
        next_states = torch.FloatTensor(np.array([e[3] for e in batch])).to(self.device)
        dones = torch.FloatTensor(np.array([e[4] for e in batch])).unsqueeze(1).to(self.device)
        
        # Update critic
        with torch.no_grad():
            # Sample next action
            next_actions, next_log_probs = self.actor(next_states, deterministic=False, with_logprob=True)
            
            # Target Q-values
            target_q1, target_q2 = self.critic_target(next_states, next_actions)
            target_q = torch.min(target_q1, target_q2)
            target_q = rewards + self.gamma * (1 - dones) * (target_q - self.alpha * next_log_probs)
        
        # Current Q-values
        current_q1, current_q2 = self.critic(states, actions)
        
        # Critic loss
        critic_loss = F.mse_loss(current_q1, target_q) + F.mse_loss(current_q2, target_q)
        
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic.parameters(), 1.0)
        self.critic_optimizer.step()
        
        # Update actor
        new_actions, log_probs = self.actor(states, deterministic=False, with_logprob=True)
        q1, q2 = self.critic(states, new_actions)
        q = torch.min(q1, q2)
        
        actor_loss = (self.alpha * log_probs - q).mean()
        
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 1.0)
        self.actor_optimizer.step()
        
        # Update alpha
        alpha_loss = -(self.log_alpha * (log_probs + self.target_entropy).detach()).mean()
        
        self.alpha_optimizer.zero_grad()
        alpha_loss.backward()
        self.alpha_optimizer.step()
        
        self.alpha = torch.exp(self.log_alpha).detach()
        
        # Soft update target networks
        self.soft_update(self.critic_target, self.critic)
        
        self.total_steps += 1
        
        return {
            'loss/critic': critic_loss.item(),
            'loss/actor': actor_loss.item(),
            'loss/alpha': alpha_loss.item(),
            'alpha': self.alpha.item()
        }
    
    def soft_update(self, target, source):
        """Soft update target network"""
        for target_param, param in zip(target.parameters(), source.parameters()):
            target_param.data.copy_(
                target_param.data * (1.0 - self.tau) + param.data * self.tau
            )
    
    def save(self, filename):
        """Save model"""
        torch.save({
            'actor_state_dict': self.actor.state_dict(),
            'critic_state_dict': self.critic.state_dict(),
            'critic_target_state_dict': self.critic_target.state_dict(),
            'actor_optimizer_state_dict': self.actor_optimizer.state_dict(),
            'critic_optimizer_state_dict': self.critic_optimizer.state_dict(),
            'alpha_optimizer_state_dict': self.alpha_optimizer.state_dict(),
            'log_alpha': self.log_alpha,
            'total_steps': self.total_steps,
            'config': {
                'gamma': self.gamma,
                'tau': self.tau,
                'alpha': self.alpha.item() if hasattr(self.alpha, 'item') else self.alpha,
                'batch_size': self.batch_size
            }
        }, filename)
        print(f"✅ Saved model to {filename}")
    
    def load(self, filename):
        """Load model"""
        if os.path.exists(filename):
            checkpoint = torch.load(filename, map_location=self.device)
            
            self.actor.load_state_dict(checkpoint['actor_state_dict'])
            self.critic.load_state_dict(checkpoint['critic_state_dict'])
            self.critic_target.load_state_dict(checkpoint['critic_target_state_dict'])
            
            self.actor_optimizer.load_state_dict(checkpoint['actor_optimizer_state_dict'])
            self.critic_optimizer.load_state_dict(checkpoint['critic_optimizer_state_dict'])
            self.alpha_optimizer.load_state_dict(checkpoint['alpha_optimizer_state_dict'])
            
            self.log_alpha = checkpoint['log_alpha']
            self.total_steps = checkpoint['total_steps']
            
            print(f"✅ Loaded model from {filename}")
            print(f"   Total steps: {self.total_steps}")
        else:
            print(f"⚠️  Model file not found: {filename}")

# Simple test
if __name__ == "__main__":
    # Create agent
    agent = FixedSAC6D()
    
    # Test action
    test_state = np.random.randn(6)
    action = agent.get_action(test_state, deterministic=False)
    print(f"Test state: {test_state}")
    print(f"Action: {action}")
    print(f"Action range: [{action.min():.3f}, {action.max():.3f}]")
    
    # Save model
    agent.save("test_model_6d.pth")