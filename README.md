# Speedpilot ROS 2 Backend

This is the ROS 2 backend for the Speedpilot project – a system for controlling Raspberry Pi-based vehicles via a mobile app.  
It handles WebSocket communication with the app, executes driving commands in real time, streams Lidar data, and enables basic obstacle avoidance using onboard sensors and algorithms.

This project was developed as part of a student thesis at DHBW Ravensburg.

---

## 📊 Project Stats & Info

<p align="center">
    <img src="https://img.shields.io/github/v/release/Rufffy99/speedpilot_ros_backend?style=flat-square" />
  <img src="https://img.shields.io/github/stars/Rufffy99/speedpilot_ros_backend?style=flat-square" />
  <img src="https://img.shields.io/github/last-commit/Rufffy99/speedpilot_ros_backend?style=flat-square" />
  <img src="https://img.shields.io/github/issues/Rufffy99/speedpilot_ros_backend?style=flat-square" />
  <img src="https://img.shields.io/github/license/Rufffy99/speedpilot_ros_backend?style=flat-square" />
</p>

---

## 🛠️ Built With

<p align="center">
  <img src="https://img.shields.io/badge/ROS%202-Jazzy-blue?style=flat-square&logo=ros" alt="ROS 2 Jazzy" />
  <img src="https://img.shields.io/badge/Python-3.12.3-yellow?style=flat-square&logo=python" alt="Python 3.12.3" />
  <img src="https://img.shields.io/badge/C++-13.3.0-blue?style=flat-square&logo=c%2B%2B" alt="C++ g++ 13.3.0" />
  <img src="https://img.shields.io/badge/CMake-3.28.3-lightgrey?style=flat-square&logo=cmake" alt="CMake 3.28.3" />
  <img src="https://img.shields.io/badge/Docker-Containerized-lightblue?style=flat-square&logo=docker" alt="Docker" />
</p>

---

## ✨ Features

- 🔌 WebSocket-based communication with the Speedpilot mobile app
- 🎮 Real-time execution of vehicle commands
- 🧠 Integrated obstacle avoidance using Lidar input
- 🔄 Continuous data exchange between ROS 2 nodes
- 🧱 Modular structure for easy expansion and debugging

---

### 🔗 Related Projects

- 📱 [Speedpilot Mobile App (Frontend)](https://github.com/tobiassng/speedpilot) – Companion app for sending commands and receiving vehicle feedback

---

## ⚙️ Installation

This project uses Docker and Docker Compose for easy development and deployment.  
Just follow the steps below to get everything up and running:

### 🐳 1. Clone the Repository

```bash
git clone https://github.com/Rufffy99/speedpilot_ros_backend.git
cd speedpilot_ros_backend
```

### 🔨 2. Build the Docker Image

```bash
docker compose build
```

### ▶️ 3. Start the Development Container

```bash
docker compose up
```

That’s it – your ROS 2 backend is now running inside a container 🚀
You can now access the container, build the workspace, and start working with your nodes.

### 🔧 4. Inside the Container – Build the ROS 2 Workspace (first time only)

```bash
cd /root/dev_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

ℹ️ This project is developed using ROS 2 Jazzy.
Be sure the correct ROS base image is used in your Dockerfile if you modify it.

---

## 🚀 Usage

Once the container is running and the workspace is built, you can start the main launch file:

```bash
source /root/dev_ws/install/setup.bash
ros2 launch speedpilot_backend bringup.launch.py
```

This will:
	•	Start all necessary backend nodes
	•	Open WebSocket communication to the Speedpilot mobile app
	•	Initialize Lidar and obstacle avoidance

📱 Interfacing with the App

Once the backend is running, the Speedpilot mobile app will automatically connect via WebSocket and begin sending commands.
You should see logs in the terminal showing command reception and Lidar data streaming.

🧪 Test the System

You can manually send commands for testing via:
```bash
ros2 topic pub /vehicle/cmd std_msgs/String "data: 'FORWARD'"
```

Replace FORWARD with LEFT, RIGHT, or STOP as needed.

---

## 📁 Project Structure

```
speedpilot_ros_backend/
├── docker/                     # Dockerfile, Compose & helper scripts
│   ├── Dockerfile
│   ├── docker-compose.yml
│   └── workspace.sh
├── entrypoint.sh               # Entrypoint for Docker container
├── workspace.sh                # Helper for dev environments
├── ros2_ws/
│   └── src/
│       ├── car_controller/             # Node handling incoming drive commands
│       ├── ros2_bridge/                # WebSocket server node (app ↔ backend)
│       ├── sllidar_ros2/               # Lidar driver and launch files
│       ├── custom_msgs/                # Custom ROS 2 message definitions
│       ├── slam_toolbox/               # SLAM algorithm implementation (forked & patched)
│       └── car_system_launch.py        # Combined system launcher
│ 
├── README.md                   # You're reading it ;)
```

> 💡 Planned nodes are marked clearly so that future contributors can easily pick up where development left off.

---

## 🤝 Contributing

This project was developed as part of a student thesis at **DHBW Ravensburg**.  
If you're a fellow student continuing this work – welcome aboard! 👋

You're invited to contribute if you:

- Want to extend the backend with new features
- Need to fix bugs or improve performance
- Plan to integrate new sensors or communication methods

### 📌 How to Contribute

1. Fork this repository
2. Create a new branch for your changes
3. Work inside the Docker container (`docker compose up`)
4. Make sure everything builds and runs (`colcon build`)
5. Submit a Pull Request with a clear description

### 🧭 Guidelines

- Keep code modular and well-commented
- Stick to ROS 2 conventions (`package.xml`, `setup.py`, launch files, etc.)
- Use meaningful commit messages
- Add tests if applicable
- Be kind in code reviews 😊

If you have questions, feel free to open an issue or start a discussion.

---

This project was developed as part of a thesis at DHBW Ravensburg.  
We hope future students can build on it and drive it even further!
