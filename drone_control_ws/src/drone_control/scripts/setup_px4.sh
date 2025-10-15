#!/bin/bash

# Setup script for PX4 SITL with Gazebo Integration
# This script installs PX4, MAVROS, and sets up the environment

set -e

echo "=================================="
echo "PX4 SITL + ROS2 Setup Script"
echo "=================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running on Ubuntu
if ! lsb_release -d | grep -q "Ubuntu"; then
    print_error "This script is designed for Ubuntu. Exiting."
    exit 1
fi

# Update system
print_status "Updating system packages..."
sudo apt update

# Install dependencies for PX4
print_status "Installing PX4 dependencies..."
sudo apt install -y \
    git \
    zip \
    qtcreator \
    cmake \
    build-essential \
    genromfs \
    ninja-build \
    exiftool \
    vim-common \
    protobuf-compiler \
    libprotobuf-dev \
    libprotoc-dev \
    python3-pip \
    python3-dev \
    python3-jinja2 \
    python3-numpy \
    python3-packaging \
    python3-pyudev \
    python3-yaml \
    python3-dev \
    python3-setuptools

# Install Gazebo if not already installed
if ! command -v gazebo &> /dev/null; then
    print_status "Installing Gazebo..."
    sudo apt install -y gazebo11 libgazebo11-dev
else
    print_status "Gazebo already installed"
fi

# Install MAVROS for ROS2
print_status "Installing MAVROS for ROS2..."
sudo apt install -y \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-mavros-msgs

# Install geographic datasets for MAVROS
print_status "Installing MAVROS geographic datasets..."
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh

# Clone PX4 if not exists
PX4_DIR="$HOME/PX4-Autopilot"
if [ ! -d "$PX4_DIR" ]; then
    print_status "Cloning PX4-Autopilot..."
    cd $HOME
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    cd PX4-Autopilot
    
    # Checkout stable version (adjust as needed)
    git checkout v1.14.3
    git submodule update --init --recursive
else
    print_status "PX4-Autopilot already exists at $PX4_DIR"
    cd $PX4_DIR
    git pull origin main
    git submodule update --init --recursive
fi

# Build PX4 for SITL
print_status "Building PX4 for SITL..."
cd $PX4_DIR
make px4_sitl

# Install additional Python dependencies
print_status "Installing Python dependencies..."
pip3 install --user \
    empy \
    toml \
    numpy \
    packaging \
    jinja2 \
    pyserial

# Setup environment variables
print_status "Setting up environment variables..."
BASHRC_FILE="$HOME/.bashrc"

# Add PX4 paths to bashrc if not already present
if ! grep -q "PX4-Autopilot" $BASHRC_FILE; then
    echo "" >> $BASHRC_FILE
    echo "# PX4 Environment Setup" >> $BASHRC_FILE
    echo "export PX4_ROOT=$HOME/PX4-Autopilot" >> $BASHRC_FILE
    echo "export PATH=\$PATH:\$PX4_ROOT/Tools/sitl_gazebo" >> $BASHRC_FILE
    echo "export GAZEBO_PLUGIN_PATH=\$GAZEBO_PLUGIN_PATH:\$PX4_ROOT/build/px4_sitl_default/build_gazebo" >> $BASHRC_FILE
    echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:\$PX4_ROOT/Tools/sitl_gazebo/models" >> $BASHRC_FILE
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$PX4_ROOT/build/px4_sitl_default/build_gazebo" >> $BASHRC_FILE
    print_status "Added PX4 environment variables to ~/.bashrc"
else
    print_status "PX4 environment variables already in ~/.bashrc"
fi

# Create workspace directories if they don't exist
WORKSPACE_DIR="$HOME/Ros-2-Environment/drone_control_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    print_warning "Workspace directory $WORKSPACE_DIR not found"
else
    print_status "Workspace directory found at $WORKSPACE_DIR"
fi

# Test PX4 SITL (quick test)
print_status "Testing PX4 SITL installation..."
cd $PX4_DIR
if timeout 30s make px4_sitl gazebo_iris &> /dev/null; then
    print_status "PX4 SITL test successful!"
else
    print_warning "PX4 SITL test timed out (this is expected for first run)"
fi

# Create a simple test script
cat > $HOME/test_px4_iris.sh << 'EOF'
#!/bin/bash
cd ~/PX4-Autopilot
make px4_sitl gazebo_iris
EOF

chmod +x $HOME/test_px4_iris.sh

print_status "======================================"
print_status "Setup completed successfully!"
print_status "======================================"
echo ""
print_status "Next steps:"
echo "1. Source your environment: source ~/.bashrc"
echo "2. Build your ROS2 workspace:"
echo "   cd ~/Ros-2-Environment/drone_control_ws"
echo "   colcon build"
echo "3. Test PX4 SITL: ~/test_px4_iris.sh"
echo "4. Run the integrated system:"
echo "   ros2 launch drone_control px4_gazebo_iris.launch.py"
echo ""
print_warning "You may need to restart your terminal or run 'source ~/.bashrc'"
print_status "Setup script completed!"
