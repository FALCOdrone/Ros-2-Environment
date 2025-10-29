## INSTRUCTIONS FOR RUNNING VISUAL SLAM

Before proceeding with the following instructions, ensure that you have installed locally the mid-air dataset from:
    https://midair.ulg.ac.be/download.html


1. **Create a python virtual environment**:
   You can create a virtual environment using the following command:
   ```
   cd visual_odom_test
   python -m venv vio_env
   ```
   Activate the virtual environment:
   - On Windows (check if works):
     ```
     vio_env\Scripts\activate
     ```
   - On macOS and Linux:
     ```
     source vio_env/bin/activate
     ```

2. **Install required packages**:
    Make sure you have `pip` installed inside the virtual environment. Then, install the required packages using:
   ```
   cd visual_odom_test
   pip install -r requirements.txt
   ```

3. **Run the Visual SLAM script**:
   You can run the visual SLAM script using the following command:
   ```
   cd visual_odom_test
   python3 visual_slam.py
   ```