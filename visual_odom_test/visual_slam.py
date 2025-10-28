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

# previous frame storage
prev_kp = None
prev_des = None
prev_img = None

# feature database (not huge memory now)
all_keypoints = []
all_descriptors = []
first_image = False


# ----------- Processing Loop -----------

for i, (left_path, right_path) in enumerate(pair_paths):
    t = timestamps[i]

    img_L = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
    img_R = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)  # loaded but unused (same behavior as original)

    if img_L is None or img_R is None:
        print(f"[WARN] skipping frame {i}, could not load")
        continue

    kp, des = feature_extractor.detectAndCompute(img_L, None)

    # Store first frame descriptors for later matching
    if i == 0:
        first_kp = kp
        first_des = des
        first_image = img_L.copy()

    # Store 20th frame descriptors for later matching
    if i == 20:
        twenty_kp = kp
        twenty_des = des
        twenty_image = img_L.copy()

    frame_points = []
    frame_des = []

    if prev_des is not None and des is not None and len(des) > 0:

        raw_matches = bf.knnMatch(prev_des, des, k=2) # using KNN (k nearest neighbors) matcher -> probably needs to be optimized

        # Lowe's ratio test, we are getting better matches if the descriptors have low distance
        good_matches = [m for m, n in raw_matches if m.distance < 0.75 * n.distance]

        for m in good_matches:
            frame_points.append(kp[m.trainIdx].pt)
            frame_des.append(des[m.trainIdx])

        print(f"[{i:04d}] t={t:.3f}s  matches: {len(good_matches)}")

        # Visualize matches only for the first 2 frames
        if i < 2 and prev_img is not None and prev_kp is not None and len(good_matches) > 0:
            try:
                img_matches = cv2.drawMatches(prev_img, prev_kp, img_L, kp, good_matches, None,
                                              matchColor=(0,255,0), singlePointColor=(255,0,0))
            except:
                img_prev_kp = cv2.drawKeypoints(prev_img, prev_kp, None, color=(0,255,0))
                img_cur_kp  = cv2.drawKeypoints(img_L, kp, None, color=(0,255,0))
                img_matches = np.hstack((img_prev_kp, img_cur_kp))

            cv2.imshow(f"Frame {i:04d}", scale_to_fit(img_matches))
            cv2.waitKey(0)

        # When we reach frame 20, compute first ↔ 20 matches
        if i == 20 and first_des is not None and twenty_des is not None:

            raw_matches_20 = bf.knnMatch(first_des, twenty_des, k=2)
            matches_20 = [m for m, n in raw_matches_20 if m.distance < 0.75 * n.distance]

            print(f"[MATCH CHECK] First frame vs 20th frame: {len(matches_20)} good matches")

            try:
                img_matches_20 = cv2.drawMatches(
                    first_image, first_kp,
                    twenty_image, twenty_kp,
                    matches_20, None,
                    matchColor=(0,255,0),
                    singlePointColor=(255,0,0)
                )
            except:
                img_first_kp = cv2.drawKeypoints(first_image, first_kp, None, color=(0,255,0))
                img_twenty_kp = cv2.drawKeypoints(twenty_image, twenty_kp, None, color=(0,255,0))
                img_matches_20 = np.hstack((img_first_kp, img_twenty_kp))

            cv2.imshow("First vs 20th Frame Matching", scale_to_fit(img_matches_20))
            cv2.waitKey(0)
        else:
            print(f"[{i:04d}] t={t:.3f}s  (first frame / no matches)")

    all_keypoints.append(frame_points)
    all_descriptors.append(frame_des)

    prev_kp = kp
    prev_des = des
    prev_img = img_L.copy()

dt = time.time() - start_time
print(f"--- Done: {len(pair_paths)} frames in {dt:.2f}s ---")
