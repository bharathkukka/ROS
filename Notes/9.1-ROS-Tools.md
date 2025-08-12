
# 🤖 ROS 2 Tools & Libraries (2025 Edition)

The **Robot Operating System (ROS)** ecosystem is packed with tools 🛠️ and libraries 📚 that make building robots faster, more reliable, and way more fun.

* **Tools** → Programs you run to build, inspect, debug, and visualize your system.
* **Libraries** → Code you import into your own nodes to add ready-made functionality.

---

## 🛠️ ROS 2 Tools

These are the programs and CLI utilities you’ll use daily to run, test, and visualize your robot’s software.

### 🔹 Core Command-Line Tools

Your main entry point: `ros2`

* **`ros2 run`** ▶️ Run a single ROS node from a package.
* **`ros2 launch`** 🚀 Start multiple nodes and configs from a launch file.
* **`ros2 topic`** 📡 Inspect topics (list, echo data, publish test messages).
* **`ros2 node`** 🔍 Get details about running nodes.
* **`ros2 service` / `ros2 action`** 🤝 List & call services or send goals to actions.
* **`ros2 param`** ⚙️ View or set node parameters at runtime.

---

### 🖥️ Graphical User Interface (GUI) Tools

Visual feedback = faster debugging.

* **RViz2** 🖼️ — The 3D visualizer for models, sensor data, and planned paths.
* **RQt** 🔌 — Modular GUI plugin framework:

  * **`rqt_graph`** 🌐 — See how nodes and topics connect.
  * **`rqt_plot`** 📊 — Plot live numeric topic data.
  * **`rqt_console`** 🗂️ — View and filter log messages.

---

### 🧪 Simulation & Data Tools

* **Gazebo / Ignition** 🏗️ — Full physics simulator for robots & environments.
* **`ros2 bag`** 🎥 — Record and replay topic data for testing.
* **Simulation Bridges (`ros_gz`)** 🌉 — Connect ROS 2 with Gazebo/Ignition for bidirectional data flow.

---

### 🛡️ Specialized & Advanced Tools

* **`rosbridge_suite`** 🌐 — WebSocket bridge to connect ROS to browsers & non-ROS apps.
* **`ros2_control` CLI** ⚙️ — Manage controllers & hardware drivers at runtime.
* **Security (SROS2)** 🔒 — Key generation, access control, and encryption tools.
* **`launch_testing`** 🧪 — Automated multi-node integration testing.

---

## 📚 ROS 2 Libraries

These are the code packages you’ll use to build nodes in C++ or Python.

---

### 🔹 Core Client Libraries

* **`rclcpp`** 💻 — ROS 2 C++ API.
* **`rclpy`** 🐍 — ROS 2 Python API.

---

### 🤖 Key Robotics Frameworks

* **`tf2`** 🧭 — Manage coordinate frames & transforms in 3D space.
* **`MoveIt 2`** 🦾 — Motion planning for arms (kinematics, collision avoidance).
* **`Navigation2`** 🚗 — Localization, mapping, path planning, and obstacle avoidance.
* **`ros2_control`** ⚙️ — Standard framework for hardware abstraction & control loops.

---

### 👁️ Perception & Data Processing

* **`cv_bridge`** 📷 — Convert between ROS images and OpenCV.
* **`image_transport`** 🗜️ — Efficient image streaming with compression.
* **`perception_pcl`** 🪨 — Point Cloud Library integration for LiDAR/depth data.

---

### 🛡️ Reliability & Diagnostics

* **`diagnostic_updater`** 🩺 — Standardized health/status reporting for nodes.
* **`robot_monitor`** 📟 — View full system health in one place.

---

### 🧩 Specialized Libraries

* **`micro-ROS`** 🔬 — Run ROS 2 on microcontrollers (ESP32, STM32, Arduino).
* **Simulation Bridges (`ros_gz`)** 🌉 — Link ROS 2 with simulation engines.

---

## 📋 Summary Table

| Category             | Examples                                                           |
| -------------------- | ------------------------------------------------------------------ |
| **CLI Tools**        | `ros2 run`, `ros2 launch`, `ros2 topic`, `ros2 node`, `ros2 param` |
| **GUI Tools**        | RViz2, RQt (`rqt_graph`, `rqt_plot`, `rqt_console`)                |
| **Simulation**       | Gazebo, Ignition, ros\_gz bridge                                   |
| **Core Libraries**   | rclcpp, rclpy                                                      |
| **Motion Planning**  | MoveIt 2, Navigation2                                              |
| **Transforms**       | tf2                                                                |
| **Perception**       | cv\_bridge, image\_transport, perception\_pcl                      |
| **Hardware Control** | ros2\_control                                                      |
| **Web/Integration**  | rosbridge\_suite                                                   |
| **Security**         | SROS2                                                              |
| **Diagnostics**      | diagnostic\_updater, robot\_monitor                                |
| **Microcontrollers** | micro-ROS                                                          |
| **Testing**          | launch\_testing                                                    |

