# drone_env.py
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pybullet as p
import pybullet_data 
import math

# Import our existing simulation components
from environment import SimulationEnvironment
from drone import Quadcopter # Assuming Quadcopter is the default

class DroneEnv(gym.Env):
    """Custom Environment for Drone Navigation that follows gym interface."""
    metadata = {"render_modes": ["human"], "render_fps": 60}

    def __init__(self, render_mode="human", drone_type=Quadcopter, obstacle_pos=[2.5, 0, 0.5], goal_pos=[5, 0, 1]):
        super().__init__()

        self.render_mode = render_mode
        self.sim = SimulationEnvironment(drone_type=drone_type, obstacle_pos=obstacle_pos, goal_pos=goal_pos)
        self.max_steps_per_episode = 240 * 15 # Max steps for 15 seconds at 240Hz
        self.current_step = 0

        # Define action space: [forward_thrust_factor, upward_thrust_factor]
        # Let's keep it simple for now, matching the Quadcopter.apply_action expectation
        # Values between -1 and 1 for normalized control
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)

        # Define observation space: 
        # For simplicity: [drone_x, drone_y, drone_z, goal_rel_x, goal_rel_y, goal_rel_z, obs_rel_x, obs_rel_y, obs_rel_z]
        # More complex spaces could include velocity, orientation, sensor data etc.
        # Box bounds should roughly match expected simulation limits
        low_obs = np.array([-10, -10, 0, -10, -10, -5, -10, -10, -5], dtype=np.float32)
        high_obs = np.array([10, 10, 10, 10, 10, 5, 10, 10, 5], dtype=np.float32)
        self.observation_space = spaces.Box(low=low_obs, high=high_obs, dtype=np.float32)

        # Connect to PyBullet if rendering
        if self.render_mode == "human":
            self.sim.connect()
            self.sim.setup_scene()
            # Assets will be loaded in reset()

    def _get_obs(self):
        """Helper function to get the current observation."""
        if self.sim.drone is None or self.sim.obstacle is None:
             # Return a zero observation if simulation isn't fully ready
             return np.zeros(self.observation_space.shape, dtype=np.float32)
             
        drone_pos, _ = self.sim.drone.get_state()
        if drone_pos is None:
             # Should not happen if drone is loaded, but handle defensively
             return np.zeros(self.observation_space.shape, dtype=np.float32)
             
        goal_rel_pos = [self.sim.goal_pos[i] - drone_pos[i] for i in range(3)]
        obs_rel_pos = [self.sim.obstacle.base_pos[i] - drone_pos[i] for i in range(3)]
        
        obs = np.array([
            drone_pos[0], drone_pos[1], drone_pos[2],
            goal_rel_pos[0], goal_rel_pos[1], goal_rel_pos[2],
            obs_rel_pos[0], obs_rel_pos[1], obs_rel_pos[2]
        ], dtype=np.float32)
        return obs

    def _get_info(self):
        """Helper function to get additional info (distance to goal)."""
        if self.sim.drone is None: return {"distance_to_goal": float("inf")}
        drone_pos, _ = self.sim.drone.get_state()
        if drone_pos is None: return {"distance_to_goal": float("inf")}
        dist = math.sqrt(sum([(self.sim.goal_pos[i] - drone_pos[i])**2 for i in range(3)]))
        return {"distance_to_goal": dist}

    def reset(self, seed=None, options=None):
        """Resets the environment to an initial state."""
        super().reset(seed=seed)
        self.current_step = 0

        # If running headless (no GUI), connect here
        if self.render_mode != "human" and not self.sim.is_connected():
            self.sim.physics_client = p.connect(p.DIRECT) # Connect in DIRECT mode for headless
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            self.sim.setup_scene() # Need to setup scene in DIRECT mode too

        # Reset simulation state (remove old bodies, reload assets)
        if self.sim.is_connected():
            # Clean up previous run if any
            if self.sim.drone and self.sim.drone.id is not None:
                try: p.removeBody(self.sim.drone.id) 
                except: pass # Ignore error if body already removed
            if self.sim.obstacle and self.sim.obstacle.id is not None:
                try: p.removeBody(self.sim.obstacle.id)
                except: pass
            if self.sim.goal_marker_id is not None:
                try: p.removeBody(self.sim.goal_marker_id)
                except: pass
            self.sim.drone = None
            self.sim.obstacle = None
            self.sim.goal_marker_id = None
            
            # Load assets for the new episode
            self.sim.load_assets()
        else:
             # This case should ideally not be reached if connect logic is correct
             print("Warning: reset called but not connected to simulation.")

        observation = self._get_obs()
        info = self._get_info()

        # Render if in human mode
        if self.render_mode == "human":
            self._render_frame()

        return observation, info

    def step(self, action):
        """ Executes one time step within the environment. """
        if not self.sim.is_connected() or self.sim.drone is None:
            # Handle case where simulation isn't ready (e.g., after an error)
            # Return a default state and indicate termination
            print("Warning: step called but simulation not ready.")
            return self._get_obs(), 0, True, True, self._get_info() # obs, reward, terminated, truncated, info

        # Apply action to the drone
        self.sim.drone.apply_action(action)

        # Step the simulation
        p.stepSimulation()
        self.current_step += 1

        # Get observation and info
        observation = self._get_obs()
        info = self._get_info()

        # Calculate reward
        distance_to_goal = info["distance_to_goal"]
        # Simple reward: negative distance to goal (encourage getting closer)
        # More complex rewards needed for better behavior
        reward = -distance_to_goal 

        # Check for termination conditions
        terminated = False
        if distance_to_goal < 0.3: # Goal reached
            reward += 100 # Bonus for reaching goal
            terminated = True
            print("Goal Reached in RL Env!")

        # Simple collision check (adds penalty)
        contact_points = p.getContactPoints(bodyA=self.sim.drone.id, bodyB=self.sim.obstacle.id)
        if len(contact_points) > 0:
            reward -= 50 # Penalty for collision
            terminated = True
            print("Collision Detected in RL Env!")




        # Check for truncation (episode length exceeded)
        truncated = False
        if self.current_step >= self.max_steps_per_episode:
            truncated = True
            print("Episode truncated (max steps reached).")

        # Render if in human mode
        if self.render_mode == "human":
            self._render_frame()

        # Gymnasium expects: observation, reward, terminated, truncated, info
        return observation, reward, terminated, truncated, info

    def render(self):
        """ Renders the environment. Currently handled by PyBullet GUI connection. """
        # In 'human' mode, PyBullet handles rendering automatically via p.connect(p.GUI)
        # If you wanted other modes (like 'rgb_array'), you'd implement image capture here.
        if self.render_mode == "human":
             # Already handled by GUI connection
             return None 
        # elif self.render_mode == "rgb_array":
             # return self._render_frame_rgb()
        pass
        
    def _render_frame(self):
        # Minimal delay to make rendering visible
        # time.sleep(1./self.metadata["render_fps"])
        pass # PyBullet GUI updates automatically

    def close(self):
        """ Closes the environment and disconnects from PyBullet. """
        self.sim.disconnect()

# Optional: Register the environment with Gymnasium
# gym.register(
#      id="DroneNav-v0",
#      entry_point="drone_env:DroneEnv",
#      max_episode_steps=240*15, # Or use the value from __init__
# )

