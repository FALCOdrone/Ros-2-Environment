#!/bin/bash

# Container-internal setup script for enhanced x500 model
# Run this script from INSIDE the Docker container

echo "Setting up enhanced X500 model (container internal)..."

# Configuration
MODELS_DIR="/workspace/models"
PX4_MODELS_PATH="/opt/px4_source/Tools/simulation/gz/models"

# Function to copy model within container
setup_model_internal() {
    echo "Copying enhanced x500 model to PX4 models directory..."
    
    if [ -d "$MODELS_DIR/x500_enhanced" ]; then
        cp -r "$MODELS_DIR/x500_enhanced" "$PX4_MODELS_PATH/"
        
        if [ $? -eq 0 ]; then
            echo "✅ Enhanced x500 model copied successfully!"
            echo "Model location: $PX4_MODELS_PATH/x500_enhanced"
            
            # List contents to verify
            echo "Model contents:"
            ls -la "$PX4_MODELS_PATH/x500_enhanced/"
        else
            echo "❌ Failed to copy model"
            return 1
        fi
    else
        echo "❌ Source model not found at $MODELS_DIR/x500_enhanced"
        echo "Available models in workspace:"
        ls -la "$MODELS_DIR/" 2>/dev/null || echo "Models directory not found"
        return 1
    fi
}

# Function to verify base x500 model exists
check_base_model() {
    echo "Checking for base x500 model..."
    if [ -d "$PX4_MODELS_PATH/x500" ]; then
        echo "✅ Base x500 model found"
    else
        echo "❌ Base x500 model not found at $PX4_MODELS_PATH/x500"
        echo "Available PX4 models:"
        ls "$PX4_MODELS_PATH/" | grep x500 || echo "No x500 models found"
    fi
}

# Function to update Gazebo environment
setup_environment() {
    echo "Setting up Gazebo environment..."
    export GZ_SIM_RESOURCE_PATH="$PX4_MODELS_PATH:/opt/px4_source/Tools/simulation/gz/worlds"
    export PX4_GZ_MODELS="$PX4_MODELS_PATH"
    
    echo "Environment variables set:"
    echo "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
    echo "PX4_GZ_MODELS=$PX4_GZ_MODELS"
}

# Main execution
echo "Enhanced X500 Container Setup"
echo "============================="
echo ""

check_base_model
echo ""
setup_model_internal
echo ""
setup_environment

echo ""
echo "🎯 Setup complete!"
echo ""
echo "📋 Next steps:"
echo "1. Launch Gazebo with enhanced model:"
echo "   ./launch_x500_enhanced.sh"
echo ""
echo "2. In another terminal, start ROS2 bridge:"
echo "   ./gz_ros2_bridge.sh"
echo ""
echo "3. Test sensor topics:"
echo "   gz topic -l | grep x500_enhanced"
