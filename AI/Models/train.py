#!/usr/bin/env python3
"""
REVERSE-TRAINING ENVIRONMENT - MULTITHREADED ACTIVE LEARNING VERSION
Multi-threaded training with proper JSON serialization
"""

import numpy as np
import torch
import random
import os
import sys
import time
import json
from collections import deque
from datetime import datetime
import threading
import multiprocessing as mp
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
import queue

# Add parent directory to path
sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

# Import enhanced SAC implementation
try:
    from SAC import ReverseReadySAC
    print("✅ Successfully imported ReverseReadySAC")
except ImportError as e:
    print(f"❌ Import error: {e}")
    sys.exit(1)

class NumpyEncoder(json.JSONEncoder):
    """Custom JSON encoder for numpy types"""
    def default(self, obj):
        if isinstance(obj, (np.integer, np.int64, np.int32)):
            return int(obj)
        elif isinstance(obj, (np.floating, np.float64, np.float32)):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, (np.bool_, bool)):
            return bool(obj)
        elif isinstance(obj, (datetime)):
            return obj.isoformat()
        return super().default(obj)

class ParallelEnvironmentRunner:
    """Runs multiple environments in parallel using threads"""
    def __init__(self, num_envs=4, curriculum_level=0):
        self.num_envs = num_envs
        self.envs = []
        self.states = []
        self.dones = []
        self.episode_rewards = [0] * num_envs
        self.episode_steps = [0] * num_envs
        self.scenario_types = []
        
        # Create environments
        for i in range(num_envs):
            env = ReverseTrainingEnv(
                enable_stuck_scenarios=True, 
                curriculum_level=curriculum_level,
                env_id=i
            )
            self.envs.append(env)
            self.states.append(env.reset())
            self.dones.append(False)
            self.scenario_types.append(env.scenario_type)
        
        self.lock = threading.Lock()
    
    def reset_env(self, env_idx):
        """Reset a specific environment"""
        with self.lock:
            self.states[env_idx] = self.envs[env_idx].reset()
            self.dones[env_idx] = False
            self.episode_rewards[env_idx] = 0
            self.episode_steps[env_idx] = 0
            self.scenario_types[env_idx] = self.envs[env_idx].scenario_type
        return self.states[env_idx]
    
    def step_env(self, env_idx, action):
        """Take a step in a specific environment"""
        with self.lock:
            next_state, reward, done, info = self.envs[env_idx].step(action)
            self.states[env_idx] = next_state
            self.dones[env_idx] = done
            self.episode_rewards[env_idx] += reward
            self.episode_steps[env_idx] += 1
        return next_state, reward, done, info

class ReverseTrainingEnv:
    def __init__(self, enable_stuck_scenarios=True, curriculum_level=0, env_id=0):
        self.env_id = env_id
        self.state_dim = 8
        self.action_dim = 6
        self.max_steps = 400
        self.enable_stuck_scenarios = enable_stuck_scenarios
        self.curriculum_level = curriculum_level
        
        # Speed factors
        self.FORWARD_SPEED_FACTOR = 5.0
        self.REVERSE_SPEED_FACTOR = 3.0
        self.MIN_DISTANCE = 5.0
        self.MAX_DISTANCE = 300.0
        
        # Dynamic difficulty
        self.difficulty_score = 0
        self.success_streak = 0
        
        # Statistics
        self.reverse_usage_stats = deque(maxlen=100)
        self.collision_stats = deque(maxlen=100)
        self.success_stats = deque(maxlen=100)
        
        # Performance tracking
        self.episode_count = 0
        self.total_steps = 0
        
        self.reset()
    
    def reset(self):
        self.step_count = 0
        self.difficulty_score = 0
        self.collision_risk = 0
        self.episode_count += 1
        
        # Curriculum-based scenario generation
        if self.curriculum_level == 0:
            self._generate_easy_scenario()
        elif self.curriculum_level == 1:
            self._generate_medium_scenario()
        else:
            self._generate_hard_scenario()
        
        # State variables
        self.steering = 90
        self.eye = 31.5
        self.obstacle_distance = self.distance
        self.distance_history = deque([self.distance] * 10, maxlen=10)
        self.reverse_used_this_episode = False
        self.forward_used_this_episode = False
        self.collision_avoided = False
        
        return self._get_state()
    
    def _generate_easy_scenario(self):
        self.distance = random.uniform(80, 200)
        self.speed = random.uniform(10, 40)
        self.obstacle_angle = random.uniform(-20, 20)
        self.scenario_type = "NORMAL"
    
    def _generate_medium_scenario(self):
        r = random.random()
        if r < 0.3:
            self.distance = random.uniform(15, 35)
            self.speed = random.uniform(-5, 15)
            self.scenario_type = "STUCK"
        elif r < 0.6:
            self.distance = random.uniform(20, 45)
            self.speed = random.uniform(5, 25)
            self.obstacle_angle = random.choice([-60, 60])
            self.scenario_type = "DEAD_END"
        else:
            self.distance = random.uniform(40, 150)
            self.speed = random.uniform(0, 40)
            self.obstacle_angle = random.uniform(-45, 45)
            self.scenario_type = "NORMAL"
    
    def _generate_hard_scenario(self):
        r = random.random()
        if r < 0.4:
            self.distance = random.uniform(8, 25)
            self.speed = random.uniform(-15, 10)
            self.scenario_type = "CRITICAL_STUCK"
        elif r < 0.8:
            self.distance = random.uniform(10, 30)
            self.speed = random.uniform(-10, 15)
            self.obstacle_angle = random.choice([-90, 90])
            self.scenario_type = "TIGHT_DEAD_END"
        else:
            self.distance = random.uniform(30, 100)
            self.speed = random.uniform(-5, 30)
            self.obstacle_angle = random.uniform(-80, 80)
            self.scenario_type = "COMPLEX"
    
    def _get_state(self):
        state = np.zeros(8, dtype=np.float32)
        state[0] = min(1.0, self.distance / 200.0)
        state[1] = np.clip(self.speed / 100.0, -1.0, 1.0)
        state[2] = (self.steering - 90) / 50.0
        state[3] = (self.eye - 31.5) / 15.0
        
        if len(self.distance_history) >= 2:
            trend = (self.distance - self.distance_history[-2]) / 10.0
            state[4] = np.clip(trend, -1.0, 1.0)
        else:
            state[4] = 0.0
        
        state[5] = 1.0 if self.distance < 40.0 else 0.0
        
        if len(self.distance_history) >= 3:
            risk = (self.distance_history[-1] - self.distance) / 3.0
            state[6] = np.clip(risk, -1.0, 1.0)
        else:
            state[6] = 0.0
        
        state[7] = np.sin(self.step_count * 0.05)
        
        return state
    
    def step(self, action):
        self.step_count += 1
        self.total_steps += 1
        
        steering_norm = np.clip(action[0], -1, 1)
        throttle_norm = np.clip(action[1], -1, 1)
        eye_norm = np.clip(action[2], -1, 1) if len(action) > 2 else 0
        
        if throttle_norm < -0.1:
            self.reverse_used_this_episode = True
        if throttle_norm > 0.1:
            self.forward_used_this_episode = True
        
        steering_delta = steering_norm * 5
        throttle_delta = throttle_norm * 12
        eye_delta = eye_norm * 3
        
        self.steering = max(40, min(140, self.steering + steering_delta))
        self.speed = max(-80, min(80, self.speed + throttle_delta))
        self.eye = max(15, min(45, self.eye + eye_delta))
        
        speed_factor = abs(self.speed) / 100.0
        if self.speed > 5:
            self.distance -= speed_factor * self.FORWARD_SPEED_FACTOR
        elif self.speed < -5:
            self.distance += speed_factor * self.REVERSE_SPEED_FACTOR
        
        self.distance += random.uniform(-0.5, 0.5)
        self.distance = max(self.MIN_DISTANCE, min(self.MAX_DISTANCE, self.distance))
        
        self.distance_history.append(self.distance)
        
        if len(self.distance_history) >= 3:
            self.collision_risk = max(0, (40 - self.distance) / 40) * \
                                 (1 if self.speed > 0 else 0.5)
        
        reward, info = self._calculate_reward(steering_norm, throttle_norm)
        
        done = False
        if self.step_count >= self.max_steps:
            done = True
            reward += 10.0
        if self.distance <= self.MIN_DISTANCE + 2:
            done = True
            self.collision_stats.append(1)
            reward -= 50.0
        else:
            self.collision_stats.append(0)
        
        if self.distance > 80 and self.distance < 200 and self.step_count > 50:
            self.success_stats.append(1)
            self.success_streak += 1
        else:
            self.success_stats.append(0)
            self.success_streak = max(0, self.success_streak - 0.5)
        
        self.reverse_usage_stats.append(1 if self.reverse_used_this_episode else 0)
        
        return self._get_state(), reward, done, info
    
    def _calculate_reward(self, steering_norm, throttle_norm):
        reward = 0.0
        info = {}
        
        if self.distance < 20:
            reward -= 5.0
            if self.speed < -10:
                reward += 3.0
            elif self.speed > 10:
                reward -= 5.0
        elif self.distance < 40:
            reward -= 2.0
            if self.speed < -5:
                reward += 2.0
            elif self.speed > 15:
                reward -= 2.0
        elif 60 < self.distance < 150:
            reward += 2.0
            if self.speed > 20:
                reward += 1.0
            if abs(steering_norm) < 0.3:
                reward += 0.5
        elif self.distance > 200:
            reward -= 1.0
        
        if len(self.distance_history) >= 5:
            distance_change = self.distance - self.distance_history[-5]
            if self.distance < 40:
                if distance_change > 5:
                    reward += 5.0
                elif distance_change > 2:
                    reward += 2.0
            elif self.distance > 100:
                if abs(distance_change) < 3:
                    reward += 1.0
        
        if self.scenario_type in ["STUCK", "CRITICAL_STUCK", "DEAD_END", "TIGHT_DEAD_END"]:
            if self.reverse_used_this_episode:
                reward += 2.0
                if self.distance > self.distance_history[0]:
                    reward += 3.0
        
        info['reward_breakdown'] = {
            'distance': float(reward),
            'reverse_used': bool(self.reverse_used_this_episode)
        }
        
        return float(reward), info

class AsyncDataCollector:
    """Asynchronous data collection using multiple threads"""
    def __init__(self, agent, num_workers=4):
        self.agent = agent
        self.num_workers = num_workers
        self.data_queue = queue.Queue(maxsize=10000)
        self.stop_event = threading.Event()
        self.workers = []
        self.collected_episodes = 0
        self.lock = threading.Lock()
    
    def start(self):
        """Start worker threads"""
        for i in range(self.num_workers):
            worker = threading.Thread(target=self._worker_loop, args=(i,))
            worker.daemon = True
            worker.start()
            self.workers.append(worker)
        print(f"✅ Started {self.num_workers} data collection workers")
    
    def stop(self):
        """Stop all workers"""
        self.stop_event.set()
        for worker in self.workers:
            worker.join(timeout=1.0)
    
    def _worker_loop(self, worker_id):
        """Worker thread main loop"""
        env = ReverseTrainingEnv(enable_stuck_scenarios=True, curriculum_level=0, env_id=worker_id)
        
        while not self.stop_event.is_set():
            state = env.reset()
            episode_data = []
            done = False
            step = 0
            
            while not done and step < env.max_steps:
                # Get action from agent (thread-safe)
                action = self.agent.get_action_async(state)
                next_state, reward, done, info = env.step(action)
                
                episode_data.append({
                    'state': state.tolist() if isinstance(state, np.ndarray) else state,
                    'action': action.tolist() if isinstance(action, np.ndarray) else action,
                    'reward': float(reward),
                    'next_state': next_state.tolist() if isinstance(next_state, np.ndarray) else next_state,
                    'done': bool(done),
                    'used_reverse': bool(action[1] < -0.1),
                    'scenario_type': env.scenario_type
                })
                
                state = next_state
                step += 1
            
            # Queue the episode data
            try:
                self.data_queue.put(episode_data, timeout=1.0)
                with self.lock:
                    self.collected_episodes += 1
            except queue.Full:
                pass

def train_reverse_ready_agent_multithreaded():
    """Multi-threaded training with active learning"""
    print("=" * 80)
    print("🚗 MULTITHREADED REVERSE-READY SAC TRAINING")
    print("=" * 80)
    
    # Create timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Setup directories
    base_dir = '/home/pi/Desktop/Intelligent-Vehicle-Navigation'
    models_dir = os.path.join(base_dir, 'AI', 'Models')
    checkpoint_dir = os.path.join(models_dir, f'checkpoints_{timestamp}')
    logs_dir = os.path.join(base_dir, 'AI', 'Logs')
    
    for directory in [models_dir, checkpoint_dir, logs_dir]:
        os.makedirs(directory, exist_ok=True)
        print(f"📁 Directory ready: {directory}")
    
    # Training parameters
    num_updates = 50000  # Number of training updates
    eval_interval = 100
    save_interval = 25  # Save every 25 updates
    batch_size = 256
    
    # Initialize agent
    agent = ReverseReadySAC(
        batch_size=batch_size,
        auto_alpha=True,
        use_per=True  # Use Prioritized Experience Replay
    )
    
    # Start async data collector
    num_cpus = mp.cpu_count()
    num_workers = min(8, num_cpus)  # Use up to 8 workers
    collector = AsyncDataCollector(agent, num_workers=num_workers)
    collector.start()
    
    print(f"\n🎯 Training for {num_updates} updates...")
    print(f"🤖 Using {num_workers} parallel environments")
    print(f"📝 Logs: {logs_dir}")
    print(f"💾 Checkpoints every {save_interval} updates to: {checkpoint_dir}")
    print("=" * 80)
    
    # Training stats
    update_times = deque(maxlen=100)
    critic_losses = deque(maxlen=100)
    actor_losses = deque(maxlen=100)
    collected_episodes_history = []
    
    try:
        for update in range(num_updates):
            update_start = time.time()
            
            # Collect batch from queue
            batch_data = []
            while len(batch_data) < batch_size and not collector.data_queue.empty():
                try:
                    episode = collector.data_queue.get_nowait()
                    # Flatten episode into transitions
                    for transition in episode:
                        batch_data.append((
                            np.array(transition['state']),
                            np.array(transition['action']),
                            transition['reward'],
                            np.array(transition['next_state']),
                            transition['done'],
                            transition['used_reverse'],
                            2.0 if transition['scenario_type'] in ['STUCK', 'CRITICAL_STUCK'] else 1.0
                        ))
                except queue.Empty:
                    break
            
            # If we don't have enough data, do a quick collection
            if len(batch_data) < batch_size:
                # Collect some experiences synchronously
                env = ReverseTrainingEnv(enable_stuck_scenarios=True)
                for _ in range(5):
                    state = env.reset()
                    done = False
                    while not done:
                        action = agent.get_action(state)
                        next_state, reward, done, info = env.step(action)
                        priority = 2.0 if env.scenario_type in ['STUCK', 'CRITICAL_STUCK'] else 1.0
                        batch_data.append((state, action, reward, next_state, done, 
                                         action[1] < -0.1, priority))
                        state = next_state
                        if len(batch_data) >= batch_size:
                            break
            
            # Perform multiple updates per collected batch for efficiency
            num_updates_this_iter = min(5, len(batch_data) // (batch_size // 2))
            
            for _ in range(num_updates_this_iter):
                # Sample batch
                if len(batch_data) > batch_size:
                    batch_indices = random.sample(range(len(batch_data)), batch_size)
                    batch = [batch_data[i] for i in batch_indices]
                else:
                    batch = batch_data[:batch_size]
                
                # Store in agent's replay buffer
                for exp in batch:
                    state, action, reward, next_state, done, used_reverse, priority = exp
                    agent.store_transition(
                        state, action, reward, next_state, done,
                        used_reverse=used_reverse,
                        priority=priority
                    )
                
                # Update agent
                stats = agent.update(reverse_priority=True)
                
                if stats:
                    critic_losses.append(stats.get('critic_loss', 0))
                    actor_losses.append(stats.get('actor_loss', 0))
            
            update_time = time.time() - update_start
            update_times.append(update_time)
            
            collected_episodes_history.append(collector.collected_episodes)
            
            # Progress reporting
            if (update + 1) % eval_interval == 0:
                avg_critic_loss = np.mean(critic_losses) if critic_losses else 0
                avg_actor_loss = np.mean(actor_losses) if actor_losses else 0
                episodes_collected = collector.collected_episodes
                updates_per_second = 1.0 / np.mean(update_times) if update_times else 0
                
                print(f"\r\033[K", end="")
                print(f"Update {update+1:6d} | "
                      f"Critic: {avg_critic_loss:8.4f} | "
                      f"Actor: {avg_actor_loss:8.4f} | "
                      f"Episodes: {episodes_collected:6d} | "
                      f"Speed: {updates_per_second:5.1f} upd/s")
                
                # Sample action test
                if update > 100:
                    test_state = np.random.randn(8).astype(np.float32)
                    test_action = agent.get_action(test_state, deterministic=True)
                    throttle_status = "REVERSE" if test_action[1] < -0.1 else "FORWARD" if test_action[1] > 0.1 else "STOP"
                    print(f"      └─ Sample throttle: {test_action[1]:+.3f} ({throttle_status})")
                
                sys.stdout.flush()
            
            # Save checkpoint
            if (update + 1) % save_interval == 0:
                checkpoint_path = os.path.join(checkpoint_dir, f'model_update{update+1}.pth')
                
                # Save with proper JSON serialization
                metadata = {
                    'update': int(update + 1),
                    'avg_critic_loss': float(avg_critic_loss) if 'avg_critic_loss' in locals() else 0.0,
                    'episodes_collected': int(collector.collected_episodes),
                    'timestamp': timestamp,
                    'update_speed': float(updates_per_second) if 'updates_per_second' in locals() else 0.0
                }
                
                # Save model
                agent.save(checkpoint_path, metadata)
                
                # Also save latest
                latest_path = os.path.join(models_dir, 'reverse_ready_model_latest.pth')
                agent.save(latest_path, metadata)
                
                print(f"\n   💾 Checkpoint saved - Update {update+1}")
    
    except KeyboardInterrupt:
        print("\n\n🛑 Training interrupted by user")
    
    finally:
        # Clean shutdown
        collector.stop()
        
        # Final save
        final_path = os.path.join(models_dir, f'reverse_ready_model_final_{timestamp}.pth')
        final_metadata = {
            'total_updates': int(num_updates),
            'total_episodes': int(collector.collected_episodes),
            'timestamp': timestamp,
            'final_critic_loss': float(np.mean(critic_losses)) if critic_losses else 0.0,
            'final_actor_loss': float(np.mean(actor_losses)) if actor_losses else 0.0
        }
        
        agent.save(final_path, final_metadata)
        print(f"\n✅ Final model saved to: {final_path}")
        
        # Save training summary
        summary_path = os.path.join(logs_dir, f'training_summary_{timestamp}.json')
        with open(summary_path, 'w') as f:
            json.dump(final_metadata, f, indent=2, cls=NumpyEncoder)
        print(f"📊 Training summary saved to: {summary_path}")
    
    return agent

def test_reverse_agent_multithreaded(agent, num_tests=10):
    """Comprehensive testing with multiple scenarios"""
    print("\n🧪 RUNNING COMPREHENSIVE TESTS...")
    
    test_results = {}
    
    for scenario in ['NORMAL', 'STUCK', 'DEAD_END', 'CRITICAL_STUCK']:
        print(f"\n--- Testing scenario: {scenario} ---")
        env = ReverseTrainingEnv(enable_stuck_scenarios=True)
        
        # Force scenario
        if scenario == 'STUCK':
            env.distance = random.uniform(10, 20)
            env.speed = random.uniform(-5, 5)
        elif scenario == 'DEAD_END':
            env.distance = random.uniform(15, 25)
            env.speed = random.uniform(5, 15)
            env.obstacle_angle = 45
        elif scenario == 'CRITICAL_STUCK':
            env.distance = random.uniform(5, 12)
            env.speed = random.uniform(-10, 0)
        
        state = env.reset()
        total_reward = 0
        steps = 0
        reverse_used = False
        
        for step in range(150):
            action = agent.get_action(state, deterministic=True)
            state, reward, done, _ = env.step(action)
            total_reward += reward
            steps += 1
            
            if action[1] < -0.1:
                reverse_used = True
            
            if step % 30 == 0:
                throttle_status = "REVERSE" if action[1] < -0.1 else "FORWARD" if action[1] > 0.1 else "STOP"
                print(f"Step {step:3d} | Distance: {env.distance:6.2f} | Speed: {env.speed:6.2f} | {throttle_status}")
            
            if done:
                break
        
        test_results[scenario] = {
            'total_reward': float(total_reward),
            'steps': int(steps),
            'reverse_used': bool(reverse_used),
            'final_distance': float(env.distance)
        }
        
        print(f"Result: Reward={total_reward:.2f}, Steps={steps}, Reverse Used={reverse_used}")
    
    # Save test results
    base_dir = '/home/pi/Desktop/Intelligent-Vehicle-Navigation'
    logs_dir = os.path.join(base_dir, 'AI', 'Logs')
    os.makedirs(logs_dir, exist_ok=True)
    
    test_path = os.path.join(logs_dir, f'test_results_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json')
    with open(test_path, 'w') as f:
        json.dump(test_results, f, indent=2, cls=NumpyEncoder)
    print(f"\n📊 Test results saved to: {test_path}")
    
    return test_results

if __name__ == "__main__":
    # Train with multi-threading
    trained_agent = train_reverse_ready_agent_multithreaded()
    
    if trained_agent:
        test_reverse_agent_multithreaded(trained_agent)
        
        print("\n" + "=" * 80)
        print("✅ TRAINING COMPLETE!")
        print("=" * 80)
    else:
        print("\n❌ Training failed - check errors above")