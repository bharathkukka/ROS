# Installing ROS 2 Humble on Ubuntu

Follow these steps to install ROS 2 Humble on Ubuntu:

## Step 1: Set Locale
Ensure your system locale is set to UTF-8:
```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

## Step 2: Add the ROS 2 Repository
First, ensure your system has `curl` installed:
```bash
sudo apt update
sudo apt install curl -y
```

Then, add the ROS 2 GPG key:
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Add the ROS 2 repository:
```bash
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```

## Step 3: Update Package Index
Update your package index to include the ROS 2 repository:
```bash
sudo apt update
```

## Step 4: Install ROS 2 Humble
Install the full desktop version of ROS 2 Humble:
```bash
sudo apt install ros-humble-desktop -y
```

Alternatively, you can install a base version:
```bash
sudo apt install ros-humble-ros-base -y
```

## Step 5: Source the Setup Script
Add the ROS 2 setup script to your shell configuration:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 6: Install Dependencies for Building Packages
Install required tools and dependencies:
```bash
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update
```

## Step 7: Verify Installation
Check if ROS 2 is installed correctly:
```bash
ros2 --version
```

You can also run a demo node:
```bash
ros2 run demo_nodes_cpp talker
```

## Additional Notes
- Ensure your Ubuntu version is compatible with ROS 2 Humble (e.g., Ubuntu 22.04).
- Refer to the [official ROS 2 documentation](https://docs.ros.org/en/humble/index.html) for more details.
- If you encounter issues, check your network and repository settings.
