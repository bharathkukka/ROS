# Setting Up ROS 2 on Apple Silicon with Docker 🚀

A step-by-step guide to get ROS 2 running smoothly on your M1/M2 MacBook using Docker containers with full GUI support.

## Prerequisites: Essential Setup 🛠️

### 1. Install Docker Desktop 🐳

Docker manages your containerized ROS 2 environment.

```bash
# Visit docker.com/products/docker-desktop/
# Download Apple Silicon version
# Install like any Mac app
# Launch and verify the whale icon appears in menu bar
```

**Verify Installation:**
```bash
docker --version
# Should show: Docker version XX.X.X
```

### 2. Install XQuartz for GUI Support 🖥️

XQuartz enables graphical ROS 2 tools (RViz2, RQt) to display on your Mac.

```bash
# Install via Homebrew
brew install --cask xquartz
```

**Critical Configuration:**
1. Launch XQuartz application
2. Navigate to **XQuartz → Settings → Security**
3. ✅ Check "Allow connections from network clients"
4. **Restart XQuartz** (quit and reopen)

***

## Step 1: Pull ROS 2 Docker Image 📦

Download the official ROS 2 Humble image with desktop tools:

```bash
docker pull ros:humble-desktop
```

*This downloads ~2GB and includes ROS 2 Humble LTS with GUI tools pre-installed.*

***

## Step 2: Launch ROS 2 Container 🐳

### Get Your Mac's IP Address
```bash
ifconfig en0 | grep inet
# Look for: inet 192.168.1.XX (your IP)
```

### Start the Container
```bash
# Replace YOUR_IP_ADDRESS with actual IP (e.g., 192.168.1.15)
docker run -it --rm \
  -e DISPLAY=YOUR_IP_ADDRESS:0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --name ros2_container \
  ros:humble-desktop bash
```

**Command Breakdown:**
- `docker run -it`: Interactive terminal session
- `--rm`: Auto-cleanup when container exits
- `-e DISPLAY=`: Routes GUI to your Mac via XQuartz
- `-v /tmp/.X11-unix`: Mounts X11 socket for GUI
- `--name ros2_container`: Memorable container name
- `ros:humble-desktop bash`: Image and shell command

*Your terminal prompt changes to indicate you're inside Ubuntu!*

***

## Step 3: Test Your Setup ✅

### Test 1: Command Line Communication

**Terminal 1 (Talker):**
```bash
# Inside container
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2 (Listener):**
```bash
# New Mac terminal
docker exec -it ros2_container bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

**Expected Output:**
```
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [listener]: I heard: [Hello World: 1]
```

### Test 2: GUI Tools (The Moment of Truth!)

```bash
# In container terminal
rqt_graph
```

**Success Indicator:** A window pops up on your Mac showing the talker→listener connection graph! 🎉

***

## Development Workflow: Code on Mac, Run in Container 💻

### Shared Workspace Setup

Stop your current container and restart with folder sharing:

```bash
# Create workspace on Mac
mkdir ~/ros2_ws

# Launch container with shared folder
docker run -it --rm \
  -e DISPLAY=YOUR_IP_ADDRESS:0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/ros2_ws:/root/ros2_ws \
  --name ros2_dev \
  ros:humble-desktop bash
```

### Recommended Workflow
1. **Edit Code**: Use VS Code/your favorite editor on Mac
2. **Build & Test**: Compile and run inside container
3. **Version Control**: Git operations on Mac side
4. **GUI Tools**: Launch visualization tools from container

***

## Quick Reference Commands 📋

### Container Management
```bash
# List running containers
docker ps

# Connect additional terminals to running container
docker exec -it ros2_container bash

# Stop container
docker stop ros2_container

# Remove container (if not using --rm)
docker rm ros2_container
```

### ROS 2 Essentials (Inside Container)
```bash
# Always source first!
source /opt/ros/humble/setup.bash

# List available packages
ros2 pkg list

# Run nodes
ros2 run <package_name> <node_name>

# Launch files
ros2 launch <package_name> <launch_file>

# Topic inspection
ros2 topic list
ros2 topic echo /topic_name
```

***

## Troubleshooting 🔧

### GUI Not Working?
```bash
# Check XQuartz is running and configured
ps aux | grep X11
# Should show XQuartz process

# Verify IP address is correct
echo $DISPLAY
```

### Container Won't Start?
```bash
# Check Docker is running
docker info

# Verify image exists
docker images | grep ros
```

### Permission Issues?
```bash
# Fix X11 permissions (run on Mac)
xhost +YOUR_IP_ADDRESS
```

***

## Pro Tips 💡

1. **Alias for Quick Launch**: Add to your `~/.zshrc`:
   ```bash
   alias ros2dev='docker run -it --rm -e DISPLAY=192.168.1.15:0 -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/ros2_ws:/root/ros2_ws ros:humble-desktop bash'
   ```

2. **Auto-Source**: Add to container's `~/.bashrc`:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

3. **Multiple Workspaces**: Use different folder names for different projects:
   ```bash
   -v ~/project1_ws:/root/project1_ws
   -v ~/project2_ws:/root/project2_ws
   ```

***

## What's Next? 🎯

- **Create Custom Packages**: Build your own ROS 2 nodes
- **Explore Visualization**: Try RViz2 for 3D robot visualization  
- **Hardware Integration**: Connect real sensors via USB passthrough
- **Advanced Networking**: Multi-machine ROS 2 communication

***
