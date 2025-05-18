import subprocess
import cv2
import numpy as np
import sys
import os

crazysim_path = os.path.expanduser("~/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/models/")


def extract_boundary(input_image, output_image='boundary_map.png', threshold=127):
    image = cv2.imread(input_image, cv2.IMREAD_GRAYSCALE)
    if image is None:
        print(f"Error: Could not read image at {input_image}")
        return

    # binary thresholding to convert to black and white
    _, binary = cv2.threshold(image, threshold, 255, cv2.THRESH_BINARY)

# Canny edge detection
    edges = cv2.Canny(binary, 50, 150)
    edges = cv2.bitwise_not(edges)
    cv2.imwrite(output_image, edges)

    print(f"Boundary map saved")
    subprocess.call(['./create_world.sh', f'{crazysim_path}'])
    
    print("Model created")

if __name__ == "__main__":
    extract_boundary()