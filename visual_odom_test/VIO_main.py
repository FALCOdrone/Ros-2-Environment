import h5py
import os
import numpy as np
import cv2


def get_camera_data(base_dir):
    """Function to gather and visualize camera data from different climate conditions."""

    # Gather all the camera directories inside each climate condition
    climate_conditions = [d for d in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, d))]
    for condition in climate_conditions:
        # Get the images inside color_down images
        color_down_dir = os.path.join(base_dir, condition, "color_down")

        # Modify accoringly to your data organization
        if condition == "foggy":
            color_down_frames_dir = os.path.join(color_down_dir, "trajectory_2000/frames")
        elif condition == "sunny":
            color_down_frames_dir = os.path.join(color_down_dir, "trajectory_0000/frames")
        else:
            print(f"Unknown condition: {condition}")
            continue

        # Check if directory exists
        if not os.path.exists(color_down_frames_dir):
            print(f"Directory not found: {color_down_frames_dir}")
            continue

        # Updated to include .JPEG extension (uppercase)
        color_images = [f for f in os.listdir(color_down_frames_dir) if f.endswith((".png", ".jpg", ".jpeg", ".JPEG"))]
        
        if not color_images:
            print(f"No image files found in {color_down_frames_dir}")
            continue
            
        print(f"Found {len(color_images)} images in {condition} condition")
        
        # Visualize images for testing
        """
        for img_name in color_images:
            img_path = os.path.join(color_down_frames_dir, img_name)
            img = cv2.imread(img_path)
            if img is not None:
                cv2.imshow(f"Color Image - {condition} - {img_name}", img)
                key = cv2.waitKey(0)
                cv2.destroyAllWindows()
                # Press 'q' to quit early
                if key == ord('q'):
                    return
            else:
                print(f"Failed to load image: {img_path}")
        """
        return color_images


# change the path to your dataset directory
base_lorenzo_dir = "/home/lorenzo/Ros-2-Environment/visual_odom_test/MidAir/MidAir/Kite_training"
get_camera_data(base_lorenzo_dir)


def process_VIO():
    """Placeholder function for VIO processing logic.

        This function will implement the visual inertial odometry processing steps.
        Will be called from the VIO class that implements the overall VIO pipeline.
    """
    pass