import cv2
import os
import time
import numpy as np

# --- Feature extractor initialization ---
try:
    from python_orb_slam3 import ORBExtractor
    try:
        feature_extractor = ORBExtractor()  # try default ORB3 wrapper
    except Exception:
        feature_extractor = cv2.ORB_create(2000)  # fallback to OpenCV
except Exception:
    feature_extractor = cv2.ORB_create(2000)

# Dataset Paths
trajectory_left_root = "/home/lorenzo/Ros-2-Environment/visual_odom_test/MidAir/MidAir/Kite_training/sunny/color_left/trajectory_0000"
trajectory_right_root = "/home/lorenzo/Ros-2-Environment/visual_odom_test/MidAir/MidAir/Kite_training/sunny/color_right/trajectory_0000"

left_files = sorted([f for f in os.listdir(os.path.join(trajectory_left_root, "frames")) if f.lower().endswith(('.jpeg', '.png'))])
right_files = sorted([f for f in os.listdir(os.path.join(trajectory_right_root, "frames")) if f.lower().endswith(('.jpeg', '.png'))])

pair_paths = [
    (os.path.join(trajectory_left_root, "frames", lf),
     os.path.join(trajectory_right_root, "frames", rf))
    for lf, rf in zip(left_files, right_files)
]

# timestamps
timestamps = [i / 25.0 for i in range(len(pair_paths))] # camera publishes at 25 Hz

print("--- Starting Processing Loop ---")
start_time = time.time()

def scale_to_fit(img, max_w=1200, max_h=800):
    h, w = img.shape[:2]
    s = min(1.0, max_w / w, max_h / h)
    return cv2.resize(img, (int(w*s), int(h*s))) if s < 1.0 else img

bf = cv2.BFMatcher(cv2.NORM_HAMMING)

# helper lists to store points and descriptors
frame_left_points = []
frame_left_des = []
frame_right_points = []
frame_right_des = []


# ----------- Processing Loop -----------

for i, (left_path, right_path) in enumerate(pair_paths):
    t = timestamps[i]

    img_L = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
    img_R = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)  # loaded but unused (same behavior as original)

    if img_L is None or img_R is None:
        print(f"[WARN] skipping frame {i}, could not load")
        continue

    # Store keypoints and descriptors for later matching
    if img_R is not None and img_L is not None:
        kp_R, des_R = feature_extractor.detectAndCompute(img_R, None)
        kp_L, des_L = feature_extractor.detectAndCompute(img_L, None)

        if des_R is not None and len(des_R) > 0 and des_L is not None and len(des_L) > 0:
            raw_matches = bf.knnMatch(des_L, des_R, k=2)
            good_matches = [m for m, n in raw_matches if m.distance < 0.75 * n.distance]
            print(f"[{i:04d}] t={t:.3f}s  stereo matches: {len(good_matches)}")

            # Append all matched keypoints and descriptors from this frame
            for m in good_matches:
                # m.queryIdx is index in des_L / kp_L, m.trainIdx is index in des_R / kp_R
                frame_left_points.append(kp_L[m.queryIdx].pt)
                frame_left_des.append(des_L[m.queryIdx])
                frame_right_points.append(kp_R[m.trainIdx].pt)
                frame_right_des.append(des_R[m.trainIdx])

            print(f"[{i:04d}] t={t:.3f}s  stereo matches: {len(good_matches)}")
        

        # Store first frame descriptors for later matching
        if i == 0:
            first_left_kp = kp_L
            first_left_des = des_L
            first_left_image = img_L.copy()
            first_right_kp = kp_R
            first_right_des = des_R
            first_right_image = img_R.copy()

        if i == 20: # example of storing 20th frame
            twenty_left_kp = kp_L
            twenty_left_des = des_L
            twenty_left_image = img_L.copy()
            twenty_right_kp = kp_R
            twenty_right_des = des_R
            twenty_right_image = img_R.copy()

        # visualize matches only dor the first 2 left and right frames
        if i < 2 and img_L is not None and kp_L is not None and len(good_matches) > 0:
            try:
                img_matches = cv2.drawMatches(img_L, kp_L, img_R, kp_R, good_matches, None,
                                              matchColor=(0,255,0), singlePointColor=(255,0,0))
            except:
                img_L_kp = cv2.drawKeypoints(img_L, kp_L, None, color=(0,255,0))
                img_R_kp  = cv2.drawKeypoints(img_R, kp_R, None, color=(0,255,0))
                img_merge_matches = np.hstack((img_L_kp, img_R_kp))
            # choose the image that was actually created for display
            display_img = img_matches if 'img_matches' in locals() else img_merge_matches
            cv2.imshow(f"Left Frame {i:04d}", scale_to_fit(display_img))
            cv2.waitKey(0)

        # When we reach frame 20, compute first ↔ 20 matches
        if i == 20 and des_L is not None and des_R is not None:
            try:
                img_matches = cv2.drawMatches(img_L, kp_L, img_R, kp_R, good_matches, None,
                                              matchColor=(0,255,0), singlePointColor=(255,0,0))
            except:
                img_L_kp = cv2.drawKeypoints(img_L, kp_L, None, color=(0,255,0))
                img_R_kp  = cv2.drawKeypoints(img_R, kp_R, None, color=(0,255,0))
                img_merge_matches = np.hstack((img_L_kp, img_R_kp))

            display_img = img_matches if 'img_matches' in locals() else img_merge_matches
            cv2.imshow(f"Left Frame {i:04d}", scale_to_fit(display_img))
            cv2.waitKey(0)

dt = time.time() - start_time
print(f"--- Done: {len(pair_paths)} frames in {dt:.2f}s ---")