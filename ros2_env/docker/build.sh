#!/bin/bash
cd ..
docker build --platform linux/amd64 -t lorenzo195815/ros2_env:latest -f docker/Dockerfile .
