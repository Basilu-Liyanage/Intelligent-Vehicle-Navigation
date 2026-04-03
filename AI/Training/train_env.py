import os
import numpy as np
import gymnasium as gym
from gymnasium import spaces

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize
from stable_baselines3.common.monitor import Monitor


# ========================= CONFIG FOR YOUR RYZEN 5 =========================
TOTAL_TIMESTEPS = 600_000          # Good for 30-60 minutes of training
N_ENVS = 6                         # Best for Ryzen 5 6-core / 12-thread
EVAL_FREQ = 25000
MODEL_PATH = r"C:\Users\user_\OneDrive\Documents\Intelligent-Vehicle-Navigation\AI\Models\sac_refined_635000_steps"
# =========================================================================


class LiDARCarEnv(gym.Env):
    """
    Original 11-observation LiDAR Car Environment (matches your old trained model)
    """
    metadata = {"render_modes": ["human"]}

    def __init__(self):
        super().__init__()

        # Observation: 11 distances (exactly as your old model)
        self.observation_space = spaces.Box(
            low=0.0, high=200.0, shape=(11,), dtype=np.float32
        )

        # Action: [throttle, steering]
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        self.max_steps = 500
        self.current_step = 0
        self.state = None

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.current_step = 0
        # Safe initial distances (as in your original)
        self.state = np.random.uniform(50, 150, size=(11,)).astype(np.float32)
        return self.state, {}

    def step(self, action):
        throttle, steer = action
        self.current_step += 1

        # Simulate LiDAR change (as in your original)
        noise = np.random.uniform(-5, 5, size=(11,))
        self.state = np.clip(self.state - throttle * 2.0 + noise, 0.0, 200.0)

        # Crash detection
        min_dist = self.state.min()
        extreme_crash = min_dist < 5.0
        crash = min_dist < 10.0

        # Reward (improved slightly for better learning while keeping your spirit)
        if extreme_crash:
            reward = -100.0
            terminated = True
        elif crash:
            reward = -50.0
            terminated = True
        else:
            reward = throttle * 1.5 + (np.mean(self.state) / 200.0) * 3.0
            terminated = False

        # Penalty for sharp steering
        reward -= abs(steer) * 0.15

        truncated = self.current_step >= self.max_steps

        return self.state.astype(np.float32), float(reward), terminated, truncated, {}


def make_env():
    env = LiDARCarEnv()
    env = Monitor(env)          # Stable with SubprocVecEnv
    return env


# ========================= MAIN =========================
if __name__ == "__main__":
    import multiprocessing
    multiprocessing.freeze_support()

    print(f"🚀 Starting Training with 11 Observations (Original Setup)")
    print(f"   → {N_ENVS} parallel environments on Ryzen 5\n")

    # Parallel environments for speed
    env = SubprocVecEnv([make_env for _ in range(N_ENVS)])
    env = VecNormalize(env, norm_obs=True, norm_reward=True)

    eval_env = SubprocVecEnv([make_env])
    eval_env = VecNormalize(eval_env, norm_obs=True, norm_reward=False, training=False)

    # Callbacks
    checkpoint_callback = CheckpointCallback(save_freq=10000, save_path="./checkpoints/", name_prefix="sac_lidar")

    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path="./best_model/",
        log_path="./eval_logs/",
        eval_freq=EVAL_FREQ,
        n_eval_episodes=3,
        deterministic=True,
        render=False
    )

    # Load or create model
    if os.path.exists(MODEL_PATH + ".zip"):
        print("🔄 Loading your old 11-obs model...")
        model = SAC.load(
            MODEL_PATH,
            env=env,
            verbose=1,
            tensorboard_log="./tensorboard/",
            ent_coef='auto_0.1'
        )
        print("✅ Old model loaded successfully (11 observations matched)")
    else:
        print("🆕 Creating new model...")
        model = SAC(
            "MlpPolicy",
            env,
            learning_rate=5e-4,
            batch_size=512,
            buffer_size=500_000,
            learning_starts=5000,
            train_freq=(1, "step"),
            gradient_steps=4,
            ent_coef='auto_0.1',
            policy_kwargs=dict(net_arch=[256, 256]),
            tensorboard_log="./tensorboard/",
            verbose=1
        )

    print("\nTraining started... Watch the FPS and eval reward!\n")
    model.learn(
        total_timesteps=TOTAL_TIMESTEPS,
        callback=[checkpoint_callback, eval_callback],
        progress_bar=False
    )

    model.save(MODEL_PATH)
    env.save("vec_normalize.pkl")
    print("\n✅ Training completed! Best model is in ./best_model/")