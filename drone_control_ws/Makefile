# Makefile for ROS2 Humble + Gazebo Docker Environment

# Configuration
IMAGE_NAME := drone_control_ros2_humble
TAG := latest
CONTAINER_NAME := drone_control_container

# Colors
RED := \033[0;31m
GREEN := \033[0;32m
YELLOW := \033[1;33m
BLUE := \033[0;34m
NC := \033[0m # No Color

.PHONY: help build start stop clean rebuild test logs shell

# Default target
help:
	@echo "$(BLUE)ROS2 Humble + Gazebo Docker Environment$(NC)"
	@echo "Available targets:"
	@echo "  $(GREEN)build$(NC)     - Build the Docker image"
	@echo "  $(GREEN)start$(NC)     - Start the container interactively"
	@echo "  $(GREEN)stop$(NC)      - Stop the running container"
	@echo "  $(GREEN)clean$(NC)     - Remove container and image"
	@echo "  $(GREEN)rebuild$(NC)   - Clean and rebuild everything"
	@echo "  $(GREEN)test$(NC)      - Test the environment setup"
	@echo "  $(GREEN)logs$(NC)      - Show container logs"
	@echo "  $(GREEN)shell$(NC)     - Get a shell in running container"
	@echo "  $(GREEN)compose-up$(NC) - Start with docker-compose"
	@echo "  $(GREEN)compose-down$(NC) - Stop docker-compose services"

# Build the Docker image
build:
	@echo "$(BLUE)Building Docker image...$(NC)"
	./docker/build.sh

# Start the container
start:
	@echo "$(BLUE)Starting container...$(NC)"
	./docker/start.sh

# Stop the container
stop:
	@echo "$(YELLOW)Stopping container...$(NC)"
	-docker stop $(CONTAINER_NAME)

# Clean up container and image
clean: stop
	@echo "$(YELLOW)Cleaning up...$(NC)"
	-docker rm $(CONTAINER_NAME)
	-docker rmi $(IMAGE_NAME):$(TAG)
	@echo "$(GREEN)Cleanup completed$(NC)"

# Rebuild everything
rebuild: clean build
	@echo "$(GREEN)Rebuild completed$(NC)"

# Test the environment
test:
	@echo "$(BLUE)Testing environment...$(NC)"
	docker exec -it $(CONTAINER_NAME) /workspace/docker/test_environment.sh || \
	echo "$(YELLOW)Container not running. Start it first with 'make start'$(NC)"

# Show container logs
logs:
	@echo "$(BLUE)Container logs:$(NC)"
	docker logs $(CONTAINER_NAME) || \
	echo "$(YELLOW)Container not found. Start it first with 'make start'$(NC)"

# Get a shell in the running container
shell:
	@echo "$(BLUE)Opening shell in container...$(NC)"
	docker exec -it $(CONTAINER_NAME) bash || \
	echo "$(YELLOW)Container not running. Start it first with 'make start'$(NC)"

# Docker Compose targets
compose-up:
	@echo "$(BLUE)Starting with docker-compose...$(NC)"
	cd docker && docker-compose up --build

compose-down:
	@echo "$(YELLOW)Stopping docker-compose services...$(NC)"
	cd docker && docker-compose down

# Build ROS2 workspace inside container
ros-build:
	@echo "$(BLUE)Building ROS2 workspace...$(NC)"
	docker exec -it $(CONTAINER_NAME) bash -c "cd /workspace && colcon build --symlink-install" || \
	echo "$(YELLOW)Container not running. Start it first with 'make start'$(NC)"

# Run specific ROS2 commands
ros-launch-sim:
	@echo "$(BLUE)Launching drone simulation...$(NC)"
	docker exec -it $(CONTAINER_NAME) bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch drone_simulation drone_simulation.launch.py" || \
	echo "$(YELLOW)Container not running or workspace not built$(NC)"

ros-rviz:
	@echo "$(BLUE)Opening RViz...$(NC)"
	docker exec -it $(CONTAINER_NAME) bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && rviz2" || \
	echo "$(YELLOW)Container not running$(NC)"

# Status check
status:
	@echo "$(BLUE)Docker status:$(NC)"
	@docker ps -a --filter "name=$(CONTAINER_NAME)" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" || true
	@echo "$(BLUE)Image status:$(NC)"
	@docker images $(IMAGE_NAME) --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}" || true
