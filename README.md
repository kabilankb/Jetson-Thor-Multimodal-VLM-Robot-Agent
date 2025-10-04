---

# 🚀 Jetson Thor Multimodal VLM Drone Agent
![Image](https://github.com/user-attachments/assets/679b9c90-3d9d-4a4f-91d6-3f6800cc0548)
### 🤖 Conversational Drone with Edge AI + Vision-Language Model (VLM)

This project demonstrates a **fully interactive multimodal drone** powered by **Jetson Thor** and the **Qwen2.5-VL-32B-Instruct-AWQ** model using **vLLM** and **ROS2**.
It enables the drone to **see, understand, and respond** in natural language — combining **computer vision**, **ROS2 robotics**, and **Edge AI**.

---

## 🧠 System Overview
<img width="1536" height="1024" alt="Image" src="https://github.com/user-attachments/assets/9044c195-c67b-40fd-911e-5595f27e4449" />
The architecture connects multiple components:

```
Web Interface ↔ ROSBridge ↔ ROS2 Nodes ↔ VLM Server (Qwen2.5-VL)
```

✅ Web interface for interaction
✅ ROS2 bridge for communication
✅ VLM model for visual-language reasoning
✅ Real-time drone perception and response

---

## ⚙️ Launching the System

Run each component in a **separate terminal** for smooth operation.

---

### **1️⃣ Start ROSBridge (WebSocket Interface)**

Allows the web client to communicate with ROS2 topics:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

### **2️⃣ Start Camera Bridge**

Converts simulation or real camera feed for web streaming:

```bash
ros2 run vlm_ros2_package simple_camera_bridge
```

---

### **3️⃣ Start the VLM Client Node**

Handles text queries and responses between the ROS2 system and the VLM model:

```bash
ros2 run vlm_ros2_package vlm_client_node
```

---

### **4️⃣ Start the Web Interface**

Opens a browser-based control and visualization dashboard:

```bash
cd ~/ros2_ws/src/vlm_ros2_package/web_interface
python app.py
```

Then open your browser and go to:

👉 [http://localhost:5000](http://localhost:5000)

---

### **5️⃣ Start the VLLM Vision-Language Model Server**

Run the **Qwen2.5-VL-32B-Instruct-AWQ** model with vLLM:

```bash
./run_vllm_vlm_serve.sh Qwen/Qwen2.5-VL-32B-Instruct-AWQ
```

---

## ✅ System Ready!

Once all terminals are running:

🧩 The drone can:

* Observe the environment through the camera
* Understand visual scenes
* Respond to queries naturally
* Enable real-time human-drone interaction

---

## 🖼️ System Diagram

```text
          ┌──────────────────────────┐
          │      Web Interface       │
          │  (User Query & Display)  │
          └────────────┬─────────────┘
                       │
              WebSocket / REST
                       │
          ┌────────────▼────────────┐
          │      ROSBridge Server   │
          │ (Connects Web ↔ ROS2)   │
          └────────────┬────────────┘
                       │
                ROS2 Topics / Nodes
                       │
          ┌────────────▼────────────┐
          │  VLM Client Node        │
          │ (Text ↔ Vision Queries) │
          └────────────┬────────────┘
                       │
                   HTTP / API
                       │
          ┌────────────▼────────────┐
          │   vLLM Server (Qwen2.5) │
          │  Vision-Language Model  │
          └────────────┬────────────┘
                       │
          ┌────────────▼────────────┐
          │     Jetson Thor Drone   │
          │ (Camera + Control Unit) │
          └─────────────────────────┘
```

---

## 📦 Repository Structure

```
Jetson-Thor-Multimodal-VLM-Robot-Agent/
│
├── ros2_ws/
│   └── src/vlm_ros2_package/
│       ├── simple_camera_bridge.py
│       ├── vlm_client_node.py
│       └── web_interface/
│           └── app.py
│
├── run_vllm_vlm_serve.sh
└── README.md
```

---

## 🧩 Technologies Used

* **NVIDIA Jetson Thor**
* **ROS2 Humble**
* **vLLM (for fast LLM inference)**
* **Qwen2.5-VL-32B-Instruct-AWQ**
* **Python + Flask Web Interface**
* **WebSocket + ROSBridge**

---

## 🧠 Reference Blog

For a detailed explanation and implementation steps, check out the full article on Medium:
📖 [Conversational Drones: Integrating Vision-Language Models on Jetson Thor for Edge AI](https://medium.com/@kabilankb2003/conversational-drones-integrating-vision-language-models-on-jetson-thor-for-edge-ai-1fae287a01dd)

---
