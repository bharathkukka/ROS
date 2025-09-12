---
# Installing ROS 2 on macOS (Apple Silicon: M1/M2/M3)

These are my personal notes for installing ROS 2 on a Mac with Apple Silicon. All steps are tested and explained simply. If you run into issues, check the official docs or community forums for updates.

---

## 1. Official Support Status

- ROS 2 supports Apple Silicon (ARM64) from Humble onwards.
- Not all packages or dependencies are ARM-native yet.
- Most core features (Python nodes, CLI tools, RViz2) work well.
- **Gazebo simulation is NOT natively supported**—use Ubuntu in a VM (Parallels, UTM) or Docker for simulation.

---

## 2. Install Homebrew (Native ARM)

If you don’t have Homebrew, open Terminal and run:

```sh
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Add Homebrew to your shell:

```sh
echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> ~/.zprofile
eval "$(/opt/homebrew/bin/brew shellenv)"
```

---

## 3. Install Dependencies (ARM)

```sh
brew install python@3.11 cmake qt5 eigen pcre poco tinyxml tinyxml2 asio poco \
  console_bridge opencv spdlog bullet tinyxml2 poco assimp \
  wget git colcon-common-extensions
```
If you see “already installed,” that’s fine.

---

## 4. Set Up Python (ARM Native)

```sh
python3.11 -m venv ~/ros2_venv
source ~/ros2_venv/bin/activate
pip install -U pip setuptools wheel
```

---

## 5. Install Rosdep

```sh
pip install rosdep
sudo rosdep init
rosdep update
```

---

## 6. Create Your ROS 2 Workspace

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

---

## 7. Clone ROS 2 Source (ARM)

Use **Jazzy Jalisco** for the latest support, or **Humble** for long-term support. Example below is for Jazzy:

```sh
cd ~/ros2_ws
git clone https://github.com/ros2/ros2.git src/ros2
cd src/ros2
git checkout jazzy
cd ../..
vcs import src < src/ros2/ros2.repos
```

---

## 8. Install More Dependencies

```sh
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
```
Replace `jazzy` with `humble` if using Humble.


---

## 9. Build ROS 2 (ARM)

This step can take 30–90 minutes, depending on your Mac. If you hit errors, copy the exact error message for troubleshooting.

```sh
colcon build --symlink-install
```

---

## 10. Source ROS in Terminal

To make ROS 2 available in every new terminal, add this to your shell config:

```sh
echo "source ~/ros2_ws/install/local_setup.zsh" >> ~/.zshrc
```

Or, for just the current terminal session:

```sh
source ~/ros2_ws/install/local_setup.zsh
```

---

## 11. Test ROS 2

Open two terminals. In both, source your ROS 2 environment:

```sh
source ~/ros2_ws/install/local_setup.zsh
```

In the first terminal, run:

```sh
ros2 run demo_nodes_cpp talker
```

In the second terminal, run:

```sh
ros2 run demo_nodes_cpp listener
```

You should see published messages from the talker to the listener!

---

## ⚠️ Apple Silicon Notes

- Some packages may not work due to lack of ARM binaries, but most core ROS 2 features and Python nodes work.
- RViz2 now works on ARM, but Gazebo simulation does not.
- For full features (especially simulation), Ubuntu in Parallels or UTM VM is recommended.
- If you need help with a package failing to build, paste the error here or check the ROS community.

---

## 12. References

- [ROS 2 on macOS (Official)](https://docs.ros.org/en/rolling/Installation/Mac-Install-Binary.html)
- [Community guide for Apple Silicon](https://discourse.ros.org/t/ros-2-on-apple-silicon-m1-m2/20638)
