# Prerequisites
WSL2 with Ubuntu 22.04
ROS2 Humble
Unity 6000.3.1f1

# Setup
1. Clone the repository:
git clone https://github.com/Sinewin/freeaskworld_instructnav_baseline.git

2. Create Conda environment from environment.yml:
conda env create -f environment.yml

3. Symbolic link for ROS2:
conda activate insnav
ln -s /home/yin/freeaskworld_closed_loop/ros2 $CONDA_PREFIX/ros2

4. Navigate to ROS2 directory and build:
cd $CONDA_PREFIX/ros2
colcon build  # Ensure dependencies are installed beforehand

5. Source the setup script:
source install/setup.bash

# Communication between WSL2 and Unity
WSL2 and Unity communicate via ROS2.
## Obtain the WSL2 IP address:
ip addr show eth0
Use the inet address for Unity configuration.

## Running the System
1. Terminal 1 (ROS2 Communication):
ros2 run ros_tcp_endpoint default_server_endpoint  # Connect to Unity
2. Terminal 2 (Run Baseline):
python instructnav_baseline.py
3. Unity: Run the scene.