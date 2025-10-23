## INSTRUCTION FOR GETTING THE MIDAIR DATASET ###

To obtain the MidAir dataset, please follow these steps:
1. Visit the official MidAir dataset website: https://midair.ulg.ac.be/download.html
2. Follow the provided instructions to download the dataset. This may involve filling out a form or agreeing to terms of use.
3. Once downloaded, extract the dataset to a desired location on your local machine.
4. Update the `base_lorenzo_dir` variable in `VIO_main.py` to point to the location where you extracted the MidAir dataset.

### Running the Odometry Code
Run the `VIO_main.py` after creating a Python virtual environment with the required dependencies installed.

The first step is to create the virtual envirnoment. You can do this by running the following commands in your terminal:

```bash
cd /path/to/Ros-2-Environment/new_model/visual_odom_test
python3 -m venv vio_env
```

Next, activate the virtual environment using the provided activation script:

```bash
source vio_env/bin/activate
```
Then, install the necessary dependencies. You can do this by running:

```bash
pip install -r requirements.txt
```

After setting up the environment and installing the dependencies, you can run the visual-inertial odometry code with the following command:

```bash
python3 VIO_main.py
```
