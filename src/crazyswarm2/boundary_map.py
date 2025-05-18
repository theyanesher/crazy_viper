import cv2
import numpy as np
import sys

def extract_boundary(input_image='/home/theya/ros2_ws/src/crazyswarm2/crazyflie_py/maps_test/1.png', output_image='boundary_map.png', threshold=127):
    """
    Convert a map image to black and white and extract only the boundary outline
    """
    # Load the image
    image = cv2.imread(input_image, cv2.IMREAD_GRAYSCALE)
    if image is None:
        print(f"Error: Could not read image at {input_image}")
        return
    
    # Apply binary thresholding to convert to black and white
    _, binary = cv2.threshold(image, threshold, 255, cv2.THRESH_BINARY)
    
    # Find edges using Canny edge detection
    edges = cv2.Canny(binary, 50, 150)
    
    # Save the edge image
    cv2.imwrite(output_image, edges)
    
    print(f"Boundary map saved as {output_image}")
    print("You can now use this image with image2gazebo")

# If running as a script
if __name__ == "__main__":
    # Use command line arguments if provided, otherwise use defaults
    input_image = sys.argv[1] if len(sys.argv) > 1 else 'map.png'
    output_image = sys.argv[2] if len(sys.argv) > 2 else 'boundary_map.png'
    threshold = int(sys.argv[3]) if len(sys.argv) > 3 else 127
    
    extract_boundary(input_image, output_image, threshold)
