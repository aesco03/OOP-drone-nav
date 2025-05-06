import pybullet as p
import pybullet_data
import time

print("Attempting to connect to PyBullet...")
try:
    # Connect to the physics server (GUI mode)
    physicsClient = p.connect(p.GUI)
    print("Connected to GUI successfully.")
    
    # Add path to PyBullet's data files
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    print(f"PyBullet data path: {pybullet_data.getDataPath()}")
    
    # Load a simple plane
    print("Loading plane...")
    planeId = p.loadURDF("plane.urdf")
    print("Plane loaded.")
    
    # Set gravity
    p.setGravity(0, 0, -9.81)
    
    print("Running simulation for a few seconds...")
    # Run simulation for a short time
    for i in range(300):
        p.stepSimulation()
        time.sleep(1./240.)
        
    print("Simulation step completed.")
    
except Exception as e:
    print(f"An error occurred: {e}")
    import traceback
    traceback.print_exc()
    
finally:
    # Disconnect
    if 'physicsClient' in locals() and p.isConnected(physicsClient):
         print("Disconnecting...")
         p.disconnect()
    print("Test finished.")

