# obstacle.py
import pybullet as p

# --- OOP: Classes & Objects --- 
# Defining a blueprint for obstacles.
class Obstacle:
    """Represents a simple obstacle in the simulation.
    Demonstrates: Class Definition, __init__ (Constructor), Encapsulation.
    """
    def __init__(self, base_pos=[5, 0, 0.5], shape_type=p.GEOM_BOX, dimensions=[1, 1, 1], color=[0.8, 0.1, 0.1, 1]):
        """Initializes the Obstacle (Constructor).

        Args:
            base_pos: Position [x, y, z]. Encapsulated attribute.
            shape_type: PyBullet geometry type. Encapsulated attribute.
            dimensions: Shape dimensions. Encapsulated attribute.
            color: RGBA color. Encapsulated attribute.
        """
        # Encapsulated instance attributes
        self.base_pos = base_pos
        self.shape_type = shape_type
        self.dimensions = dimensions
        self.color = color
        self.id = None # PyBullet body ID, managed internally
        print(f"Obstacle initialized at {base_pos}")

    # Encapsulated method
        # Encapsulated method
    def load(self):
        """Loads the obstacle into the simulation.
        Encapsulates the PyBullet shape creation and multi-body loading logic.
        """
        print(f"Creating visual and collision shapes for obstacle...")
        # Internal details of shape creation are hidden within this method

        visual_shape_id = -1
        collision_shape_id = -1

        # Explicitly handle parameters for each shape type
        if self.shape_type == p.GEOM_BOX:
            if not isinstance(self.dimensions, list) or len(self.dimensions) != 3:
                raise ValueError(f"Invalid dimensions for GEOM_BOX: {self.dimensions}. Expected [half_width, half_depth, half_height]")
            visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                                halfExtents=self.dimensions,
                                                rgbaColor=self.color)
            collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                      halfExtents=self.dimensions)
        elif self.shape_type == p.GEOM_SPHERE:
            if not isinstance(self.dimensions, list) or len(self.dimensions) != 1:
                raise ValueError(f"Invalid dimensions for GEOM_SPHERE: {self.dimensions}. Expected [radius]")
            visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE,
                                                radius=self.dimensions[0],
                                                rgbaColor=self.color)
            collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_SPHERE,
                                                      radius=self.dimensions[0])
        elif self.shape_type == p.GEOM_CYLINDER:
            if not isinstance(self.dimensions, list) or len(self.dimensions) != 2:
                raise ValueError(f"Invalid dimensions for GEOM_CYLINDER: {self.dimensions}. Expected [radius, height]")
            visual_shape_id = p.createVisualShape(shapeType=p.GEOM_CYLINDER,
                                                radius=self.dimensions[0],
                                                length=self.dimensions[1], # PyBullet uses 'length' for visual height
                                                rgbaColor=self.color)
            collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_CYLINDER,
                                                      radius=self.dimensions[0],
                                                      height=self.dimensions[1]) # PyBullet uses 'height' for collision height
        else:
            raise ValueError(f"Unsupported shape type: {self.shape_type}")


        if visual_shape_id == -1 or collision_shape_id == -1:
            # Check if shape creation failed
            raise RuntimeError(f"Failed to create obstacle shapes (visual_id={visual_shape_id}, collision_id={collision_shape_id})")

        print(f"Creating multi-body for obstacle...")
        # Mass = 0 makes it static (part of its state, encapsulated)
        self.id = p.createMultiBody(baseMass=0,
                                    baseCollisionShapeIndex=collision_shape_id,
                                    baseVisualShapeIndex=visual_shape_id,
                                    basePosition=self.base_pos)
        
        if self.id is None or self.id < 0:
            raise RuntimeError("Failed to create obstacle multi-body.")
            
        print(f"Obstacle loaded with ID: {self.id}")
        return self.id

    
#class MyNewObstacle(Obstacle):
    
    
# --- OOP: Inheritance (Example - Optional for Demo) --- 
# MovingObstacle *is an* Obstacle. It could inherit and add/override behavior.
# class MovingObstacle(Obstacle):
#     """Example of inheriting from Obstacle to create a moving variant."""
#     def __init__(self, base_pos=[5, 0, 0.5], shape_type=p.GEOM_SPHERE, dimensions=[0.5], color=[0.1, 0.8, 0.1, 1], velocity=[0, 0.1, 0]):
#         # Call base class constructor
#         super().__init__(base_pos, shape_type, dimensions, color)
#         # Add new attribute specific to MovingObstacle
#         self.velocity = velocity 
#         print("MovingObstacle type initialized.")

#     # Polymorphism: Override load to make it non-static and set velocity
#     def load(self):
#         visual_shape_id = p.createVisualShape(shapeType=self.shape_type, radius=self.dimensions[0], rgbaColor=self.color)
#         collision_shape_id = p.createCollisionShape(shapeType=self.shape_type, radius=self.dimensions[0])
#         # Override baseMass to be non-zero
#         self.id = p.createMultiBody(baseMass=1, 
#                                     baseCollisionShapeIndex=collision_shape_id,
#                                     baseVisualShapeIndex=visual_shape_id,
#                                     basePosition=self.base_pos)
#         # Add new behavior: set initial velocity
#         p.resetBaseVelocity(self.id, linearVelocity=self.velocity)
#         print(f"Moving Obstacle loaded with ID: {self.id}")
#         return self.id