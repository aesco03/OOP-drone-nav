# environment.py
import pybullet as p
import pybullet_data
import time
import math

# Import our custom classes
# Demonstrates modularity - separating concerns into different files/classes.
from drone import Quadcopter, FixedWing # Allow choosing drone type
from obstacle import Obstacle

# --- OOP: Classes & Objects --- 
# Defining a blueprint for the simulation environment.


#Manages the PyBullet simulation environment, objects, and stepping.
#Demonstrates: Class Definition, __init__, Encapsulation, Composition (contains Drone and Obstacle objects).
class SimulationEnvironment:
    
# Initializes the simulation environment (Constructor). 
# Args: drone_type: The class of the drone to use (e.g., Quadcopter). Demonstrates passing classes as arguments. 
# obstacle_pos: Position [x, y, z] for the obstacle. Encapsulated attribute. 
# goal_pos: Target position [x, y, z] for the drone. Encapsulated attribute. 

    
    def __init__(self, drone_type=Quadcopter, obstacle_pos=[3, 0, 0.5], goal_pos=[5, 0, 1]):
       
        # Encapsulated instance attributes managing the simulation state
        self.physics_client = None
        self.plane_id = None
        self.drone = None # Composition: Environment 'has a' Drone
        self.obstacle = None # Composition: Environment 'has an' Obstacle
        self.goal_pos = goal_pos
        self.goal_marker_id = None
        self.drone_type = drone_type # Stores the *class* to be instantiated
        self.obstacle_pos = obstacle_pos
        print("SimulationEnvironment initialized.")

    # Encapsulated method
    def connect(self):
        """Connects to the PyBullet physics server.
        Hides the PyBullet connection details.
        """
        print("Connecting to PyBullet GUI...")
        self.physics_client = p.connect(p.GUI)
        if self.physics_client < 0:
            raise RuntimeError("Failed to connect to PyBullet GUI.")
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        print("Connected successfully.")

    # Encapsulated method
    def setup_scene(self):
        """Sets up the basic simulation scene (gravity, plane, camera).
        Encapsulates common setup steps.
        """
        if not self.is_connected():
            print("Error: Not connected to physics server.")
            return
            
        print("Setting up scene...")
        p.setGravity(0, 0, -9.81)
        self.plane_id = p.loadURDF("plane.urdf")
        p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[2.5, 0, 0])
        print("Scene setup complete (Gravity, Plane, Camera).")

    # Encapsulated method - Demonstrates Composition
    def load_assets(self):
        """Loads the drone, obstacle, and goal marker into the environment.
        Creates instances of Drone and Obstacle classes (Composition).
        """
        if not self.is_connected():
            print("Error: Not connected to physics server.")
            return
            
        print("Loading assets...")
        # Create Drone instance using the specified type (Polymorphism via constructor argument)
        self.drone = self.drone_type(start_pos=[0, 0, 0.5]) 
        self.drone.load() # Calls the load method of the specific drone instance

        # Create Obstacle instance
        self.obstacle = Obstacle(base_pos=self.obstacle_pos, shape_type=p.GEOM_BOX, dimensions=[0.5, 0.5, 1], color=[0.8, 0.1, 0.1, 1])
        self.obstacle.load() # Calls the load method of the Obstacle instance

        # Goal marker setup (internal detail)
        goal_visual_shape = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.2, rgbaColor=[0.1, 0.8, 0.1, 0.7])
        self.goal_marker_id = p.createMultiBody(baseVisualShapeIndex=goal_visual_shape,
                                              baseCollisionShapeIndex=-1, 
                                              basePosition=self.goal_pos)
        print("Assets loaded (Drone, Obstacle, Goal Marker).")

    # Encapsulated helper method
    def is_connected(self):
        """Checks if connected to the physics server."""
        return self.physics_client is not None and p.isConnected(self.physics_client)

    # Encapsulated method - Main simulation logic
    def run_simulation(self, duration_sec=15):
        """Runs the simulation loop for a given duration.
        Encapsulates the core simulation logic, including agent interaction (placeholder).
        """
        if not self.is_connected() or self.drone is None:
            print("Error: Cannot run simulation. Ensure connection and assets are loaded.")
            return

        print(f"Running simulation for {duration_sec} seconds...")
        start_time = time.time()
        steps = 0
        max_steps = int(duration_sec * 240) 

        while steps < max_steps:
            # --- Interaction with Drone Object --- 
            # 1. Get state from the drone object (using its encapsulated method)
            drone_pos, drone_orn = self.drone.get_state()
            if drone_pos is None: break
                
            # --- Placeholder RL/Control Logic --- 
            # Calculate direction to goal
            goal_vec = [self.goal_pos[i] - drone_pos[i] for i in range(3)]
            dist_to_goal = math.sqrt(sum(x**2 for x in goal_vec))
            goal_dir = [x / dist_to_goal for x in goal_vec] if dist_to_goal > 1e-6 else [0,0,0]

            # Simple Obstacle Avoidance Logic
            obstacle_vec = [self.obstacle.base_pos[i] - drone_pos[i] for i in range(3)]
            dist_to_obstacle = math.sqrt(sum(x**2 for x in obstacle_vec))
            avoidance_thrust = [0, 0, 0]
            if dist_to_obstacle < 1.5: 
                avoidance_dir_xy = [-obstacle_vec[0], -obstacle_vec[1], 0]
                norm_xy = math.sqrt(avoidance_dir_xy[0]**2 + avoidance_dir_xy[1]**2)
                if norm_xy > 1e-6: avoidance_dir_xy = [x / norm_xy for x in avoidance_dir_xy]
                avoidance_thrust = [avoidance_dir_xy[0]*0.3, avoidance_dir_xy[1]*0.3, 0.1] 
                
            # 2. Determine action based on state (simple proportional control + avoidance)
            target_dir = [goal_dir[i] + avoidance_thrust[i] for i in range(3)]
            # Simplified action [forward_factor, upward_factor] - specific to Quadcopter.apply_action
            action = [target_dir[0] * 0.5, target_dir[2] * 0.5] 
            
            # --- Interaction with Drone Object --- 
            # 3. Apply action *to the drone object* (using its encapsulated method - Polymorphism)
            # The environment doesn't need to know *how* the drone executes the action.
            self.drone.apply_action(action, gravity_z=-9.81)

            # 4. Step the simulation (internal PyBullet detail)
            p.stepSimulation()
            
            # 5. Check conditions
            if dist_to_goal < 0.3: 
                print("Goal Reached!")
                break

            time.sleep(1./240.)
            steps += 1
            if steps % 240 == 0: print(f"Simulated {steps // 240} seconds...")

        print("Simulation finished.")

    # Encapsulated method
    def disconnect(self):
        """Disconnects from the PyBullet physics server.
        Encapsulates cleanup logic.
        """
        if self.is_connected():
            print("Disconnecting from PyBullet...")
            p.disconnect()
            self.physics_client = None
        print("Disconnected.")