#!/bin/bash

# PX4 SITL Gazebo x500 Setup Script
# This script helps you build and run the PX4 SITL simulation with Gazebo and x500 model

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if Docker is running
if ! docker info >/dev/null 2>&1; then
    print_error "Docker is not running. Please start Docker first."
    exit 1
fi

# Check if we're in the right directory
if [ ! -f "docker-compose.yml" ]; then
    print_error "docker-compose.yml not found. Please run this script from the drone_control_ws directory."
    exit 1
fi

# Function to show usage
show_usage() {
    echo "Usage: $0 [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  build       Build the Docker image for PX4 SITL with Gazebo"
    echo "  run         Run the interactive drone control container"
    echo "  px4-full    Run the fully integrated system (PX4 + Gazebo + MAVROS)"
    echo "  gazebo-only Run only Gazebo simulation"
    echo "  legacy      Run the legacy Gazebo Classic setup"
    echo "  clean       Clean up Docker containers and images"
    echo "  help        Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 build         # Build the new Gazebo image"
    echo "  $0 run           # Run interactive container"
    echo "  $0 px4-full      # Run complete integrated system"
}

case "$1" in
    "build")
        print_info "Building PX4 SITL Gazebo Docker image..."
        docker-compose build drone-control
        print_success "Docker image built successfully!"
        print_info "You can now run: $0 run"
        ;;
    
    "run")
        print_info "Starting interactive drone control container with Gazebo support..."
        print_info "Container will have access to:"
        print_info "  - PX4 SITL with x500 model"
        print_info "  - Gazebo simulation environment"
        print_info "  - MAVROS for ROS2 communication"
        print_info "  - Your ROS2 workspace"
        echo ""
        print_info "Available commands in container:"
        print_info "  /usr/local/bin/launch_px4_gz_x500.sh  - Launch PX4 with x500"
        print_info "  /usr/local/bin/launch_mavros.sh        - Launch MAVROS"
        print_info "  /usr/local/bin/launch_gz_only.sh       - Launch Gazebo only"
        echo ""
        docker-compose up drone-control
        ;;
    
    "px4-full")
        print_info "Starting fully integrated PX4 SITL system..."
        print_info "This will start: PX4 SITL + Gazebo x500 + MAVROS"
        docker-compose --profile px4-full up px4-integrated
        ;;
    
    "gazebo-only")
        print_info "Starting Gazebo simulation only..."
        docker-compose --profile gazebo-only up gazebo-gz
        ;;
    
    "legacy")
        print_info "Starting legacy Gazebo Classic setup..."
        docker-compose --profile legacy up drone-control-classic
        ;;
    
    "clean")
        print_warning "This will remove all containers and the drone control images."
        read -p "Are you sure? (y/N): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            print_info "Stopping and removing containers..."
            docker-compose down --remove-orphans
            print_info "Removing images..."
            docker rmi drone_control_ros2_gz:latest 2>/dev/null || true
            docker rmi drone_control_ros2_simple:latest 2>/dev/null || true
            print_success "Cleanup completed!"
        else
            print_info "Cleanup cancelled."
        fi
        ;;
    
    "help"|"--help"|"-h"|"")
        show_usage
        ;;
    
    *)
        print_error "Unknown command: $1"
        echo ""
        show_usage
        exit 1
        ;;
esac
