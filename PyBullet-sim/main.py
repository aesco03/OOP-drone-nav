# main.py
import sys
from environment import SimulationEnvironment
from drone import Quadcopter, FixedWing # Import drone types

def main():
    # Main function to set up and run the simulation
    print("Starting PyBullet Drone OOP Demo...")

    # --- Configuration --- 
    # Choose the drone type to use (demonstrates flexibility)
    # drone_to_use = Quadcopter 
    drone_to_use = Quadcopter 
    # drone_to_use = FixedWing # Uncomment to try the placeholder FixedWing
    
    obstacle_position = [2.5, 0, 0.5] # Position the obstacle in the middle
    goal_position = [5, 0, 1]       # Position the goal further away
    simulation_duration = 20        # Run for 20 seconds

    # --- Simulation Setup --- 
    # Create an instance of the SimulationEnvironment
    # Pass the chosen drone class and positions
    env = SimulationEnvironment(drone_type=drone_to_use, 
                              obstacle_pos=obstacle_position, 
                              goal_pos=goal_position)

    try:
        # Connect to PyBullet
        env.connect()
        # Setup the basic scene
        env.setup_scene()
        # Load the drone, obstacle, and goal marker
        env.load_assets()
        # Run the main simulation loop
        env.run_simulation(duration_sec=simulation_duration)

    except Exception as e:
        print(f"An error occurred during simulation: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # Ensure disconnection even if errors occur
        env.disconnect()
        print("Demo finished.")

if __name__ == "__main__":
    main()

