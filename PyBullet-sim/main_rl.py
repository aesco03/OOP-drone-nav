# main_rl.py
import os
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import CheckpointCallback

# Import our custom environment
from drone_env import DroneEnv 
# Import drone types if needed for environment initialization (though DroneEnv uses Quadcopter by default)
# from drone import Quadcopter, FixedWing 

# --- Configuration --- 
TRAIN_AGENT = True # Set to True to train, False to load and run demo
LOAD_MODEL_PATH = "ppo_drone_final.zip" # Path to load model from if TRAIN_AGENT is False

TOTAL_TIMESTEPS = 50000 # Reduce for quick testing, increase significantly for real training (e.g., 1,000,000+)
ALGORITHM = PPO
POLICY = "MlpPolicy" # Multi-Layer Perceptron policy (standard for continuous states/actions)
MODEL_SAVE_NAME = "ppo_drone"
LOG_DIR = "./ppo_drone_logs/"
CHECKPOINT_FREQ = 10000 # Save a checkpoint every N steps

def main():
    """Main function to train or run the RL agent."""
    print("Starting PyBullet Drone RL Demo...")
    os.makedirs(LOG_DIR, exist_ok=True)

    # --- Environment Setup --- 
    # Instantiate the custom Gym environment
    # Use render_mode="human" to see the GUI during demo/short training
    # Use render_mode=None (or another mode) for faster headless training on servers
    # env = DroneEnv(render_mode="human") 
    env = DroneEnv(render_mode=None if TRAIN_AGENT else "human") 

    # Optional: Check if the environment follows the Gym interface
    # try:
    #     print("Checking environment...")
    #     check_env(env)
    #     print("Environment check passed!")
    # except Exception as e:
    #     print(f"Environment check failed: {e}")
    #     env.close()
    #     return

    # --- RL Model Setup --- 
    if TRAIN_AGENT:
        print(f"Setting up {ALGORITHM.__name__} model for training...")
        # Define the model
        model = ALGORITHM(POLICY, env, verbose=1, tensorboard_log=LOG_DIR)
        
        # Setup checkpoint callback
        checkpoint_callback = CheckpointCallback(
            save_freq=CHECKPOINT_FREQ,
            save_path=LOG_DIR,
            name_prefix=MODEL_SAVE_NAME
        )

        # --- Training --- 
        print(f"Starting training for {TOTAL_TIMESTEPS} timesteps...")
        try:
            model.learn(total_timesteps=TOTAL_TIMESTEPS, callback=checkpoint_callback, progress_bar=True)
            # Save the final model
            final_model_path = os.path.join(LOG_DIR, f"{MODEL_SAVE_NAME}_final.zip")
            model.save(final_model_path)
            print(f"Training finished. Final model saved to {final_model_path}")
        except Exception as e:
            print(f"An error occurred during training: {e}")
            import traceback
            traceback.print_exc()
        finally:
            env.close()
            print("Training environment closed.")

    else: # Run Demo with pre-trained model
        print(f"Loading pre-trained model from {LOAD_MODEL_PATH}...")
        if not os.path.exists(LOAD_MODEL_PATH):
            print(f"Error: Model file not found at {LOAD_MODEL_PATH}")
            env.close()
            return
            
        model = ALGORITHM.load(LOAD_MODEL_PATH, env=env)
        print("Model loaded. Starting demo...")

        # --- Demo Loop --- 
        episodes = 5
        for ep in range(episodes):
            obs, info = env.reset()
            terminated = False
            truncated = False
            ep_reward = 0
            steps = 0
            while not terminated and not truncated:
                action, _states = model.predict(obs, deterministic=True) # Use deterministic actions for demo
                obs, reward, terminated, truncated, info = env.step(action)
                ep_reward += reward
                steps += 1
                # env.render() # Handled by DroneEnv if render_mode="human"
            print(f"Episode {ep+1}: Reward = {ep_reward:.2f}, Steps = {steps}, Terminated={terminated}, Truncated={truncated}")
        
        env.close()
        print("Demo finished.")

if __name__ == "__main__":
    main()

