# drone.py
import pybullet as p
import numpy as np
# --- OOP: Classes & Objects --- 
# Defining a blueprint for drones.
class Drone:
    """Base class for all drone types in the simulation.
    Represents the core properties and functionalities common to all drones.
    Demonstrates: Class Definition, __init__ (Constructor), Encapsulation (attributes like id, methods like load, get_state).
    """
    def __init__(self, start_pos=[0, 0, 1], start_orn=[0, 0, 0]):
        """Initializes the Drone base class (Constructor).

        Args:
            start_pos: Initial position [x, y, z]. Encapsulated attribute.
            start_orn: Initial orientation as Euler angles [roll, pitch, yaw]. Encapsulated attribute.
        """
        # Encapsulated instance attributes
        self.start_pos = start_pos
        self.start_orn_quat = p.getQuaternionFromEuler(start_orn) # Internal detail encapsulated
        self.urdf_path = None # To be set by derived classes (Polymorphism)
        self.id = None # PyBullet body ID, managed internally
        print(f"Base Drone initialized at {start_pos}")

    # Encapsulated method
    def load(self):
        """Loads the drone model into the simulation.
        Encapsulates the PyBullet loading logic.
        """
        if self.urdf_path is None:
            # Abstraction: Base class defines the need for a URDF, but not which one.
            raise NotImplementedError("URDF path must be set by derived class")
        
        print(f"Loading drone from: {self.urdf_path}")
        self.id = p.loadURDF(self.urdf_path,
                             self.start_pos,
                             self.start_orn_quat)
        if self.id is None:
             raise RuntimeError(f"Failed to load URDF: {self.urdf_path}")
        print(f"Drone loaded with ID: {self.id}")
        return self.id

    # Encapsulated method
    def get_state(self):
        """Gets the current state (position, orientation) of the drone.
        Hides the PyBullet API call.
        """
        if self.id is None:
            return None, None
        pos, orn = p.getBasePositionAndOrientation(self.id)
        return pos, orn

    # Encapsulated method - Abstraction
    def apply_action(self, action):
        """Applies a given action to the drone.
        Abstracts the concept of applying an action. 
        The specific implementation is deferred to subclasses (Polymorphism).
        """
        # Base implementation does nothing, forcing subclasses to override if they need control.
        # print(f"Applying action {action} to drone {self.id} (Base class - no action)")
        pass

# --- OOP: Inheritance & Polymorphism --- 
# Quadcopter *is a* Drone. It inherits properties and methods from Drone.
class Quadcopter(Drone):
    """Represents a specific type of drone: a quadcopter.
    Inherits from the Drone base class.
    Demonstrates: Inheritance (extends Drone), Polymorphism (overrides apply_action).
    """
    def __init__(self, start_pos=[0, 0, 1], start_orn=[0, 0, 0]):
        """Initializes the Quadcopter."""
        # Call the base class constructor using super()
        super().__init__(start_pos, start_orn)
        # Set the specific URDF for this drone type (Specialization)
        self.urdf_path = "entities/quadrotor.urdf" 
        print("Quadcopter type initialized.")
        
    def load(self):
        """Loads the drone model and retrieves its mass."""
        super().load() # Call the base class load method
        # Get dynamics info after loading. Index 0 is mass.
        dynamics_info = p.getDynamicsInfo(self.id, -1) 
        self.mass = dynamics_info[0]
        if self.mass <= 0:
             print(f"Warning: Drone mass reported as {self.mass}. Using default 0.5kg for hover calculation.")
             self.mass = 0.5 # Use a default if mass is invalid
        print(f"Quadcopter mass retrieved: {self.mass:.3f} kg")
        return self.id

    # Polymorphism: Overriding the base class method with improved visual flight logic
    def apply_action(self, action, gravity_z):
        """Applies forces to simulate quadcopter movement for visual demo.
        Attempts to counteract gravity and move based on simple action inputs.
        Action: [forward_thrust_factor, upward_thrust_factor] (range -1 to 1)
        """
        if self.id is None or self.mass is None:
            # Don't apply forces if not loaded or mass not known
            return
            
        base_pos, base_orn = self.get_state()
        if base_pos is None:
            return

        # Get orientation matrix and gravity
        rot_matrix = p.getMatrixFromQuaternion(base_orn)
        gravity = gravity_z # Use the provided gravity Z component


        # Calculate required hover force (must oppose gravity)
        hover_force_magnitude = abs(self.mass * gravity)

        # --- Calculate Forces based on Action --- 
        # Action[0]: Controls forward/backward thrust relative to drone orientation
        # Action[1]: Controls upward/downward thrust relative to hover
        
        # Get drone's forward vector (X-axis in drone's local frame)
        # Column 0 of the rotation matrix [R[0], R[3], R[6]]
        forward_vec = np.array([rot_matrix[0], rot_matrix[3], rot_matrix[6]])
        # Get drone's up vector (Z-axis in drone's local frame)
        # Column 2 of the rotation matrix [R[2], R[5], R[8]]
        up_vec = np.array([rot_matrix[2], rot_matrix[5], rot_matrix[8]])

        # Scale factors for thrust (adjust these if movement is too fast/slow)
        forward_thrust_scale = 3.0 * self.mass # Scale force roughly by mass
        vertical_thrust_scale = 2.0 * self.mass # Scale force roughly by mass

        # Calculate target forces
        forward_force_vec = forward_vec * action[0] * forward_thrust_scale
        # Vertical force = hover force + control input
        vertical_force_vec = up_vec * (hover_force_magnitude + action[1] * vertical_thrust_scale)

        # Combine forces
        total_force_world = forward_force_vec + vertical_force_vec

        # Apply the total force at the center of mass (linkIndex=-1) in world coordinates
        p.applyExternalForce(self.id, -1, 
                             forceObj=total_force_world.tolist(), 
                             posObj=base_pos, # Apply at center of mass
                             flags=p.WORLD_FRAME)

        # --- Optional: Add some damping --- 
        # Helps prevent excessive spinning or drifting (tune factor as needed)
        lin_vel, ang_vel = p.getBaseVelocity(self.id)
        damping_factor = 0.1 * self.mass # Scale damping roughly by mass
        p.applyExternalForce(self.id, -1, 
                             forceObj=[-v * damping_factor for v in lin_vel], 
                             posObj=base_pos, 
                             flags=p.WORLD_FRAME)
        p.applyExternalTorque(self.id, -1, 
                              torqueObj=[-av * damping_factor * 0.1 for av in ang_vel], # Lower torque damping
                              flags=p.WORLD_FRAME)

# --- OOP: Inheritance & Polymorphism --- 
# FixedWing *is a* Drone.
class FixedWing(Drone):
    """Represents a fixed-wing drone (placeholder).
    Inherits from the Drone base class.
    Demonstrates: Inheritance, Polymorphism (can override apply_action).
    """
    def __init__(self, start_pos=[0, 0, 5], start_orn=[0, 0, 0]):
        super().__init__(start_pos, start_orn)
        # Specialization: Set a different URDF
        # Using cube as placeholder as pybullet_data lacks a simple fixed-wing model
        self.urdf_path = "entities/fixed_wing.urdf" 
        print("FixedWing type initialized (using cube URDF as placeholder).")

    # Polymorphism: Overriding the base class method (optional if behavior differs)
    def apply_action(self, action):
        """Applies fixed-wing specific actions (placeholder)."""
        # Fixed-wing control logic would involve simulating lift, drag, control surfaces.
        # print(f"Applying fixed-wing action {action} (placeholder)")
        # Example: Apply forward force and maybe some lift based on orientation/speed
        base_pos, base_orn = self.get_state()
        if base_pos is None: return
        rot_matrix = p.getMatrixFromQuaternion(base_orn)
        forward_vec = [rot_matrix[0], rot_matrix[3], rot_matrix[6]]
        # Apply constant forward thrust for simplicity
        p.applyExternalForce(self.id, -1, forceObj=[f*5 for f in forward_vec], posObj=base_pos, flags=p.WORLD_FRAME)
        pass