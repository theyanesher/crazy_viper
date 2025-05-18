# #!/usr/bin/env python3

# import numpy as np
# import sys
# import time
# from crazyflie_py import Crazyswarm

# def main():
#     # Initialize Crazyswarm
#     swarm = Crazyswarm()
#     timeHelper = swarm.timeHelper
#     allcfs = swarm.allcfs
    
#     # Get available drone IDs
#     available_ids = list(allcfs.crazyfliesById.keys())
#     print(f"Available drone IDs: {available_ids}")
    
#     if not available_ids:
#         print("No drones found!")
#         return
    
#     # Track the current positions of drones (starting with initial positions)
#     current_positions = {}
#     for drone_id in available_ids:
#         cf = allcfs.crazyfliesById[drone_id]
#         current_positions[drone_id] = np.array(cf.initialPosition)
    
#     print("\nInitial positions of all drones:")
#     for drone_id, pos in current_positions.items():
#         print(f"Drone {drone_id}: Position [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
    
#     takeoff_height = 0.5  # Default takeoff height
#     print(f"\nTaking off to {takeoff_height}m...")
#     allcfs.takeoff(targetHeight=takeoff_height, duration=3.0)
#     timeHelper.sleep(3.5)
    
#     # Update current positions after takeoff
#     for drone_id in available_ids:
#         current_positions[drone_id][2] = takeoff_height
    
#     # Display positions after takeoff
#     print("\nPositions after takeoff:")
#     for drone_id, pos in current_positions.items():
#         print(f"Drone {drone_id}: Position [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
    
#     # Main control loop
#     running = True
#     while running:
#         while True:
#             try:
#                 drone_id = int(input(f"\nEnter drone ID from {available_ids}: "))
#                 if drone_id in available_ids:
#                     break
#                 else:
#                     print(f"Invalid ID. Available IDs are: {available_ids}")
#             except ValueError:
#                 print("Please enter a valid number.")
        
#         try:
#             x = float(input("Enter X coordinate: "))
#             y = float(input("Enter Y coordinate: "))
#             z = float(input("Enter Z coordinate: "))
#             yaw = float(input("Enter yaw angle (degrees, default 0): ") or 0)
#         except ValueError:
#             print("Invalid coordinate input. Using default values.")
#             x, y, z, yaw = 0.0, 0.0, 1.0, 0.0
        
#         # Convert yaw to radians
#         yaw_rad = np.radians(yaw)
        
#         target_pos = np.array([x, y, z])
#         print(f"\nSending drone {drone_id} to position [{x}, {y}, {z}] with yaw {yaw}°")
#         allcfs.crazyfliesById[drone_id].goTo(target_pos, yaw_rad, 5.0)  # Use a reasonable default duration
        
#         # Update the expected position for this drone
#         current_positions[drone_id] = target_pos
        
#         # # Wait for the movement to complete without printing positions
#         # print("Drone is moving to target position...")
#         # timeHelper.sleep(5.5)  
        
#         # Display final positions after movement
#         print("\nCurrent positions after movement:")
#         for drone_id, pos in current_positions.items():
#             print(f"Drone {drone_id}: Position [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
        
#         land = input("\nDo you want to land the drones? (y/n): ").lower() == 'y'
#         if land:
#             print("Landing...")
#             allcfs.land(targetHeight=0.02, duration=3.0)
#             timeHelper.sleep(3.5)
            
#             # Update positions after landing
#             for drone_id in available_ids:
#                 current_positions[drone_id][2] = 0.02  # Set z to landing height
            
#             # Display positions after landing
#             print("\nPositions after landing:")
#             for drone_id, pos in current_positions.items():
#                 print(f"Drone {drone_id}: Position [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
            
#             print("Done!")
#             running = False  
#         else:
#             print("Continuing with position commanding...")

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3

import numpy as np
import sys
import time
from crazyflie_py import Crazyswarm

def main():
    # Initialize Crazyswarm
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    
    # Get available drone IDs
    available_ids = list(allcfs.crazyfliesById.keys())
    print(f"Available drone IDs: {available_ids}")
    
    if not available_ids:
        print("No drones found!")
        return
    
    # Function to get current positions from Crazyswarm
    def get_positions():
        positions = {}
        for drone_id in available_ids:
            cf = allcfs.crazyfliesById[drone_id]
            # Get position from the state estimator
            pos = cf.position()  # This is the Crazyswarm position function
            print(f"Drone {drone_id} position from po: {pos}")
            positions[drone_id] = pos
        return positions
    
    # Try to get positions using the position function
    try:
        current_positions = get_positions()
        position_function_works = True
        print("Successfully using Crazyswarm position function")
    except (AttributeError, TypeError) as e:
        # Fall back to initialPosition if position() doesn't work
        print(f"Position function not available: {e}")
        position_function_works = False
        current_positions = {}
        for drone_id in available_ids:
            cf = allcfs.crazyfliesById[drone_id]
            current_positions[drone_id] = np.array(cf.initialPosition)
    
    print("\nInitial positions of all drones:")
    for drone_id, pos in current_positions.items():
        print(f"Drone {drone_id}: Position [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
    
    takeoff_height = 0.5  # Default takeoff height
    print(f"\nTaking off to {takeoff_height}m...")
    allcfs.takeoff(targetHeight=takeoff_height, duration=3.0)
    timeHelper.sleep(3.5)
    
    # Update positions after takeoff
    if position_function_works:
        current_positions = get_positions()
    else:
        # Update current positions after takeoff
        for drone_id in available_ids:
            current_positions[drone_id][2] = takeoff_height
    
    # Display positions after takeoff
    print("\nPositions after takeoff:")
    for drone_id, pos in current_positions.items():
        print(f"Drone {drone_id}: Position [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
    
    # Main control loop
    running = True
    while running:
        while True:
            try:
                drone_id = int(input(f"\nEnter drone ID from {available_ids}: "))
                if drone_id in available_ids:
                    break
                else:
                    print(f"Invalid ID. Available IDs are: {available_ids}")
            except ValueError:
                print("Please enter a valid number.")
        
        try:
            x = float(input("Enter X coordinate: "))
            y = float(input("Enter Y coordinate: "))
            z = float(input("Enter Z coordinate: "))
            yaw = float(input("Enter yaw angle (degrees, default 0): ") or 0)
        except ValueError:
            print("Invalid coordinate input. Using default values.")
            x, y, z, yaw = 0.0, 0.0, 1.0, 0.0
        
        # Convert yaw to radians
        yaw_rad = np.radians(yaw)
        
        target_pos = np.array([x, y, z])
        print(f"\nSending drone {drone_id} to position [{x}, {y}, {z}] with yaw {yaw}°")
        allcfs.crazyfliesById[drone_id].goTo(target_pos, yaw_rad, 5.0)  # Use a reasonable default duration
        
        # Wait a moment for the command to be processed
        timeHelper.sleep(0.1)
        
        # Update positions
        if position_function_works:
            # Get actual positions from Crazyswarm
            current_positions = get_positions()
        else:
            # Update the expected position for this drone
            current_positions[drone_id] = target_pos
        
        # Display current positions
        print("\nCurrent positions after sending command:")
        for drone_id, pos in current_positions.items():
            print(f"Drone {drone_id}: Position [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
        
        # Wait for movement to complete
        print("\nWaiting for movement to complete...")
        timeHelper.sleep(5.0)
        
        # Update positions after movement
        if position_function_works:
            current_positions = get_positions()
        
        # Display positions after movement
        print("\nPositions after movement:")
        for drone_id, pos in current_positions.items():
            print(f"Drone {drone_id}: Position [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
        
        land = input("\nDo you want to land the drones? (y/n): ").lower() == 'y'
        if land:
            print("Landing...")
            allcfs.land(targetHeight=0.02, duration=3.0)
            timeHelper.sleep(3.5)
            
            # Update positions after landing
            if position_function_works:
                current_positions = get_positions()
            else:
                # Update positions after landing
                for drone_id in available_ids:
                    current_positions[drone_id][2] = 0.02  # Set z to landing height
            
            # Display positions after landing
            print("\nPositions after landing:")
            for drone_id, pos in current_positions.items():
                print(f"Drone {drone_id}: Position [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
            
            print("Done!")
            running = False  
        else:
            print("Continuing with position commanding...")

if __name__ == '__main__':
    main()
