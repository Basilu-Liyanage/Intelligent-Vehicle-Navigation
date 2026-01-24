# File: Vehicle/DataLogger/record_drives.py
import pickle
import time

class ExperienceRecorder:
    def __init__(self, save_path="world_model_training_data.pkl"):
        self.buffer = []
        self.save_path = save_path
        
    def record_step(self, lidar_cm, gps_data, action_taken):
        """Record (state, action, next_state) for world model training"""
        experience = {
            'state': [lidar_cm, gps_data['lat'], gps_data['lon']],
            'action': action_taken,  # [speed, steering]
            'timestamp': time.time(),
            'episode': self.current_episode
        }
        self.buffer.append(experience)
        
    def save_and_clear(self):
        with open(self.save_path, 'wb') as f:
            pickle.dump(self.buffer, f)
        print(f"✅ Saved {len(self.buffer)} experiences to {self.save_path}")
        self.buffer = []