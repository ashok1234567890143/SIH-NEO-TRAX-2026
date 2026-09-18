# SIH-NEO-TRAX-2026
# SIH 2026 Autonomous Vehicle Prototype
## "Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads"

[![SIH 2026](https://img.shields.io/badge/SIH-2026%20Prototype-orange.svg)](https://sih.gov.in)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2020a%2B-blue.svg)](https://mathworks.com)
[![ESP32](https://img.shields.io/badge/Hardware-Dual%20ESP32-red.svg)](https://espressif.com)
[![Dashboard](https://img.shields.io/badge/Dashboard-HTML5%20%2F%20Canvas-green.svg)](file:///d:/SIH/SIH_Autonomous_Vehicle/WEB/index.html)

---

## 1. Executive Summary

Autonomous navigation on unstructured Indian roads presents fundamental challenges that standard autonomous driving stacks (designed for structured lane-marked highways) cannot handle:
1. **Absence of lane markings & irregular road boundaries** (dirt/mud shoulders, eroded asphalt edges).
2. **Dynamic heterogeneous traffic** (cows, pedestrians crossing diagonally, auto-rickshaws, motorcycles).
3. **Road surface hazards** (potholes, sudden barricades, unpaved diversions).

This prototype delivers a cost-effective, high-reliability architecture that operates **without expensive LiDAR or power-hungry NVIDIA Jetson/Raspberry Pi boards**, utilizing:
* **Dual ESP32 Embedded Architecture**: Strict hardware decoupling between vision streaming (`ESP32-CAM`) and real-time motor/ultrasonic safety control (`Main ESP32`).
* **Laptop MATLAB Brain**: Centralizes high-level computer vision, 5-channel ultrasonic sensor fusion, 2D dynamic cost mapping with Gaussian inflation, adaptive **RRT\*** and **A\*** path planning with multi-objective optimization, and Pure Pursuit differential tracking.
* **Interactive Live Web Dashboard**: High-tech HTML5/Canvas visualization showcasing real-time camera feeds, top-down road maps, sensor telemetry, and SIH demonstration test scenarios.

---

## 2. System Architecture

```
                       ┌───────────────────────────────┐
                       │           ESP32-CAM           │
                       │    "Eyes of the Vehicle"      │
                       │  • OV2640 Image Acquisition   │
                       │  • MJPEG / Capture Server     │
                       │  • Wi-Fi Auto-Reconnect       │
                       └───────────────┬───────────────┘
                                       │
                                     Wi-Fi
                               (HTTP MJPEG / JPEG)
                                       │
                                       ▼
                       ┌───────────────────────────────┐
                       │         LAPTOP (MATLAB)       │
                       │     "High-Level Intelligence" │
                       │                               │
                       │  1. Vision Perception (Edges) │
                       │  2. 5x US Array Sensor Fusion │
                       │  3. Dynamic Cost Map Buffer   │
                       │  4. Adaptive RRT* & A* Search │
                       │  5. Multi-Tier AEB Controller │
                       │  6. Pure Pursuit Motor PWM    │
                       └───────────────┬───────────────┘
                                       │
                                   USB Serial
                             (115200 Baud ASCII)
                                       │
                                       ▼
                       ┌───────────────────────────────┐
                       │       MAIN ESP32 CONTROLLER   │
                       │   "Low-Level Real-Time Reflex"│
                       │                               │
                       │  • 5x HC-SR04 Median Filter   │
                       │  • Hardware Safety Layer (AEB)│
                       │  • Dual Motor PWM Generation  │
                       │  • Wheel Encoder Counting     │
                       │  • Heartbeat Watchdog         │
                       └───────────────┬───────────────┘
                                       │
                ┌──────────────────────┼──────────────────────┐
                ▼                      ▼                      ▼
        5x Ultrasonic Array       Motor Driver           2x Encoders
      [L, FL, F, FR, R]       (L298N / TB6612)         (Optical/Hall)
                                       │
                                       ▼
                                2x DC Gear Motors
```

---

## 3. Project Directory Structure

```
SIH_Autonomous_Vehicle/
│
├── MATLAB/
│   ├── main.m                         # Master execution loop (20 Hz)
│   ├── config.m                       # Master configuration parameters
│   ├── camera/
│   │   ├── connectCamera.m            # ESP32-CAM HTTP interface
│   │   ├── getCameraFrame.m           # Frame grabber (Real / Synthetic)
│   │   └── processCameraFrame.m       # Edge detection & blob extraction
│   ├── sensors/
│   │   ├── connectESP32.m             # Serial port connector
│   │   ├── readESP32Data.m            # Serial buffer reader
│   │   ├── parseSensorData.m          # ASCII telemetry parser
│   │   └── sensorFusion.m             # Camera + Ultrasonic fusion
│   ├── perception/
│   │   ├── detectObstacles.m          # Spatial Euclidean clustering
│   │   ├── detectRoad.m               # Unstructured road boundary estimator
│   │   └── estimateObstaclePosition.m # 3D ground projection
│   ├── mapping/
│   │   ├── createCostMap.m            # 2D Grid map initialization
│   │   └── updateCostMap.m            # Gaussian obstacle inflation
│   ├── planning/
│   │   ├── adaptivePathPlanner.m      # RRT* with A* fallback
│   │   ├── generateCandidatePaths.m   # Evasive tentacle splines
│   │   ├── collisionCheck.m           # Fine-grained collision validator
│   │   ├── calculatePathCost.m        # Multi-objective cost evaluator
│   │   └── replanPath.m               # Dynamic lookahead replanner
│   ├── control/
│   │   ├── vehicleController.m        # Pure Pursuit differential controller
│   │   ├── speedController.m          # Adaptive velocity profiler
│   │   ├── steeringController.m       # Angular velocity PID
│   │   └── calculateEmergencyStop.m   # Multi-tier safety decision matrix
│   ├── communication/
│   │   ├── sendCommandToESP32.m       # Transmit motor PWM packets
│   │   └── receiveESP32Data.m         # Receive telemetry packets
│   ├── visualization/
│   │   ├── visualizeRoad.m            # Bird's-eye road visualizer
│   │   ├── visualizeVehicle.m         # Vehicle chassis & raycasts
│   │   ├── visualizePath.m            # Active & replanned path overlays
│   │   └── updateDashboardData.m      # JSON exporter for Web UI
│   ├── simulation/
│   │   ├── createRoadScenario.m       # Virtual environment setup
│   │   ├── createObstacles.m          # Indian road obstacles generator
│   │   └── simulateVehicle.m          # Unicycle physics & raycasting
│   └── metrics/
│       └── performanceMetrics.m       # Quantitative benchmark logger
│
├── ESP32/
│   └── main_controller/
│       └── main_controller.ino        # ESP32 real-time motor & US controller
│
├── ESP32_CAM/
│   └── camera_stream/
│       └── camera_stream.ino          # ESP32-CAM Wi-Fi video streamer
│
├── WEB/
│   ├── index.html                     # SIH 2026 live demonstration dashboard
│   ├── style.css                      # Glassmorphic cyber dark stylesheet
│   ├── script.js                      # Canvas renderer & simulation engine
│   └── dashboard_data.json            # Real-time JSON telemetry bridge
│
└── DOCUMENTATION/
    ├── README.md                      # This document
    ├── SYSTEM_ARCHITECTURE.md         # Deep technical specifications & equations
    ├── WIRING.md                      # Schematics, pinouts, and power supply
    └── TESTING.md                     # 13-Phase test procedure & verification guide
```

---

## 4. Quick Start Guide

### A. Run in Simulation Mode (No physical hardware required)
1. Open MATLAB and navigate to `SIH_Autonomous_Vehicle/MATLAB/`.
2. Open `config.m` and ensure `cfg.mode = 'SIMULATION'`.
3. Open `WEB/index.html` in any modern web browser (Chrome, Edge, Firefox).
4. In MATLAB, run:
   ```matlab
   main
   ```
5. **Interactive Controls & Scenario Customization**:
   * **Spawn Obstacles**: Left-click anywhere on the Road Map, or press `v` (Car), `t` (Truck), `c` (Cow), `r` (Auto), `p` (Pedestrian), `h` (Pothole), `b` (Barricade).
   * **Delete Obstacles**: Right-click on an obstacle to delete it, press `x` to delete the nearest obstacle, or press `z` to clear all obstacles.
   * **Change Destination**: Shift-click anywhere on the road or click the `Change Goal` button to relocate the destination marker dynamically!
   * **Instant Emergency Braking**: High-deceleration instant braking response ($v \rightarrow 0$ with zero latency).
   * **Switch Scenarios**: Select presets 1 through 8 (including Scenario 8: Multi-Vehicle Oncoming Rush & Avoidance to Destination).

### B. Run in Hardware Mode (Physical Prototype)
1. Flash `ESP32_CAM/camera_stream/camera_stream.ino` to the ESP32-CAM.
   * Update Wi-Fi SSID and password in the `.ino` file.
   * Note the IP address printed on the Serial Monitor (e.g., `http://192.168.4.150`).
2. Flash `ESP32/main_controller/main_controller.ino` to the Main ESP32.
3. Connect the Main ESP32 to the laptop via USB. Note the COM port (e.g., `COM3`).
4. In `MATLAB/config.m`, set:
   ```matlab
   cfg.mode = 'HARDWARE';
   cfg.serial.port = 'COM3'; % Your ESP32 COM port
   cfg.camera.url = 'http://192.168.4.150/capture';
   ```
5. Run `main.m` in MATLAB.

---

## 5. Software & Hardware Prerequisites

### Software Requirements
* **MATLAB**: R2020a or newer (Base MATLAB with standard toolboxes: Image Processing Toolbox recommended).
* **Arduino IDE**: 1.8.19 or 2.x with **ESP32 by Espressif Systems** board package installed (v2.0.x+).
* **Web Browser**: Any modern browser supporting HTML5 Canvas.

### Hardware Bill of Materials (BOM)
| Component | Purpose | Typical Cost (INR) |
|---|---|---|
| **AI-Thinker ESP32-CAM** | Vision capture & Wi-Fi video streaming | ₹450 |
| **ESP32 DevKit V1** | Real-time motor & ultrasonic controller | ₹350 |
| **5x HC-SR04 Ultrasonic Sensors** | Multi-directional distance measurement | ₹350 (5x ₹70) |
| **L298N / TB6612FNG Driver** | Differential dual DC motor driving | ₹150 |
| **2x DC Gear Motors + Wheels** | Vehicle propulsion | ₹200 |
| **Caster Wheel + 2WD Chassis** | Vehicle robotic platform | ₹250 |
| **7.4V / 11.1V Li-ion Battery** | Motor & logic power | ₹400 |
| **USB Cable + Jumper Wires** | Serial comms & interconnects | ₹100 |
| **Total Hardware Cost** | **Ultra-low budget prototype** | **~₹2,250** |

---

## 6. Multi-Objective Cost Function for Path Planning

The adaptive path planner optimizes trajectory selection using:

$$J(\pi) = w_1 \cdot L(\pi) + w_2 \cdot \text{Risk}(\pi) + w_3 \cdot \text{Clearance}(\pi) + w_4 \cdot \Delta\theta(\pi) + w_5 \cdot \Delta v(\pi)$$

Where:
* $w_1 = 1.0$: Minimizes total travel distance $L(\pi)$.
* $w_2 = 8.5$: Heavily penalizes traversing through inflated obstacle cost zones.
* $w_3 = 3.0$: Rewards maximizing lateral distance from road curbs and objects.
* $w_4 = 2.0$: Penalizes abrupt heading changes to ensure smooth differential steering.
* $w_5 = 1.5$: Prevents jerky velocity transitions.

---

## 7. SIH 2026 Demonstration Talking Points

During jury evaluation, emphasize the following key architectural strengths:
1. **Zero High-End Compute Dependency**: Achieves real-time RRT* path planning and sensor fusion without requiring expensive Jetson Nano or LiDAR.
2. **Deterministic Dual Safety Layers**: If laptop communication freezes, the Main ESP32 onboard Autonomous Emergency Braking (AEB) independently halts the vehicle within 18 cm.
3. **Unstructured Road Adaptation**: Does not assume painted highway lanes; models irregular road corridors, dust shoulders, and moving Indian traffic elements.
4. **Resilient Replanning**: Evasive tentacle bypass executes within **<15 ms**, while full RRT* global re-plan converges in **<60 ms**.
