#!/usr/bin/env python3
"""
Industry-level SAC trainer for wheelchair/car with 7 distance sensors,
speed (0-100%), and steering angle (0-50°).
"""

import numpy as np
import torch
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
        state = env.reset()
        total_reward = 0
        done = False
        while not done:
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