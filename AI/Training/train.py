# train_sac.py
import os
import sys
import gymnasium as gym
import numpy as np
from stable_baselines3 import SAC
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import BaseCallback

sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')
from train_env import LiDARCarEnv  # your custom env

# ========================
# Setup save directory
# ========================
SAVE_DIR = r"C:\Users\user_\OneDrive\Documents\Intelligent-Vehicle-Navigation\AI\Models"
os.makedirs(SAVE_DIR, exist_ok=True)
MODEL_PATH = os.path.join(SAVE_DIR, "sac_lidar_car")

# ========================
# Callback to save model periodically
# ========================
class SaveCallback(BaseCallback):
    def __init__(self, save_freq=5000, verbose=1):
        super().__init__(verbose)
        self.save_freq = save_freq

    def _on_step(self) -> bool:
        if self.num_timesteps % self.save_freq == 0:
            path = f"{MODEL_PATH}_{self.num_timesteps}_steps.zip"
            self.model.save(path)
            if self.verbose:
                print(f"Saved model at {self.num_timesteps} steps to {path}")
        return True

# ========================
# Create environment
# ========================
env = LiDARCarEnv()
check_env(env, warn=True)  # sanity check

# ========================
# Create or load model
# ========================
if os.path.exists(MODEL_PATH + ".zip"):
    print("Loading existing model...")
    model = SAC.load(MODEL_PATH, env=env, device="auto", verbose=1)
else:
    print("Creating new model...")
    model = SAC("MlpPolicy", env, verbose=1, learning_rate=0.0003,
                batch_size=256, buffer_size=100000, tau=0.005,
                gamma=0.99, train_freq=(1, "step"), device="auto")

# ========================
# Training
# ========================
TIMESTEPS = 500_000
callback = SaveCallback(save_freq=5000)

print("Starting training...")
model.learn(total_timesteps=TIMESTEPS, callback=callback)
model.save(MODEL_PATH)
print(f"Training complete. Model saved at {MODEL_PATH}.zip")

# ========================
# Test the trained model
# ========================
obs, _ = env.reset()
for i in range(100):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    print(f"Step {i+1} | Action: {action} | Reward: {reward}")
    if terminated or truncated:
        obs, _ = env.reset()