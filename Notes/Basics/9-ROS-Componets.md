
### **Filesystem Level Concepts** 💾

This level deals with how ROS resources are organized on your computer's disk.

* **Packages** 📦: The primary unit for organizing software in ROS. A package can contain ROS nodes, libraries, configuration files, and more. Think of it as a folder containing everything needed for a specific capability.
* **Package Manifests** 📝 (`package.xml`): An XML file inside every package. It defines the package's properties — name, version, author, and dependencies on other packages.
* **Metapackages** 📚: Special packages that don't contain code or files themselves but group together a set of related packages. Example: a `navigation` metapackage might bring in all the necessary individual packages for robot navigation.
* **Message Files** 📄 (`.msg`) – Define the data structures for messages exchanged between nodes (custom or standard).
* **Service Files** 🔄 (`.srv`) – Define request/response structures for services used in request/reply node interactions.
* **Action Files** 🎯 (`.action`) – Define the goal, result, and feedback structure for actions used in long-running, preemptible tasks.

---

### **Computation Graph Level Concepts** 🔗

This is the peer-to-peer network of ROS processes that work together to perform tasks — the **runtime** level of ROS.

* **Nodes** 🧠: The main computational units. Each node is a process that performs a specific task — e.g., controlling a wheel motor, processing camera data, or planning a path.
* **Master (`roscore`)** 📞: The central coordinator that helps nodes find and communicate with each other — like a phone book for the ROS system.
* **Topics** 🗣️: Named buses where nodes exchange data. Nodes **publish** messages 📤 to a topic and other nodes **subscribe** 📥 to receive them. Example: a camera node might publish images on `/camera/image`.
* **Messages** 📄 (`.msg`): Data structures sent over topics. ROS has many standard message types (e.g., for sensor data, velocity commands) and supports custom ones.
* **Services** 🔄 (`.srv`): Synchronous request/response communication. A server node provides a service; a client sends a request and waits for a reply. Example: “calculate inverse kinematics for this arm position.”
* **Actions** 🎯 (`.action`): For long-running tasks that give feedback during execution, such as navigating to a waypoint. Clients can send goals, receive progress updates, and cancel goals.
* **Parameter Server** ⚙️: A shared dictionary on the ROS Master for storing and retrieving configuration parameters at runtime.
* **Bags** 🎥 (`.bag`): Files for recording and replaying message data — great for debugging, testing algorithms, or simulating robots without hardware.

---

### **Community Level Concepts** 🌍

These are the resources and tools supporting the global ROS community.

* **Distributions (Distros)** 📅: Versioned sets of ROS packages, like Linux distros. Examples: **ROS Noetic Ninjemys** 🐢 (ROS 1) and **ROS 2 Humble Hawksbill** 🦎.
* **ROS Wiki** 📖: The official primary source for ROS documentation.
* **ROS Answers** 💬: A dedicated Q\&A forum to get help from the community.
* **Repositories** 💻: Online code hosting platforms (often GitHub) where ROS packages are developed and maintained.
* **Community Resources** 📖 – Official docs, tutorials, Q\&A forums (ROS Answers 💬), and community-driven platforms for learning and support.


---  
