# DeepSafe: FPGA-Based Edge AI Safety System for Confined Space Workers

> **An FPGA-powered Edge AI Safety System that monitors hazardous confined spaces using real-time multi-sensor fusion, hardware acceleration, and intelligent anomaly detection.**

![Platform](https://img.shields.io/badge/Platform-PYNQ--Z2-blue)
![FPGA](https://img.shields.io/badge/FPGA-Xilinx%20Zynq--7020-red)
![Language](https://img.shields.io/badge/RTL-Verilog-orange)
![Python](https://img.shields.io/badge/Python-PYNQ-green)
![License](https://img.shields.io/badge/License-MIT-success)

---

## Overview

Workers operating inside manholes, underground tunnels, sewage systems, mines, and industrial confined spaces are constantly exposed to life-threatening hazards such as:

- Toxic gas leakage
- Fire hazards
- Heat stress
- Falls or sudden depth changes

Traditional monitoring systems often depend on cloud connectivity, making them unsuitable for environments with unreliable communication.

**DeepSafe** performs all sensing, processing, and safety decisions directly on the FPGA, ensuring ultra-low latency and reliable operation without internet connectivity.

---

## Features

- Digital sensor acquisition
- Hardware-based moving average filtering
- Multi-sensor feature extraction
- FPGA-accelerated TinyML inference
- Rule-based safety engine
- Multi-sensor fusion
- Real-time anomaly detection
- AXI4-Lite configurable parameters
- Live PYNQ Jupyter dashboard
- Completely offline operation

---

# System Architecture

```
                    Sensors
         ┌───────────────────────────┐
         │ MQ2  Flame  HC-SR04 DHT11 │
         └─────────────┬─────────────┘
                       │
                Sensor Interfaces
                       │
             Windowed Moving Filters
                       │
              Feature Extraction
                       │
        ┌──────────────┴──────────────┐
        │                             │
  Threshold Engine              Tiny MLP
        │                             │
        └──────────────┬──────────────┘
                       │
              Fusion & Hybrid Voter
                       │
            Alert Generation System
                       │
         Dashboard • UART • LEDs • Relay
```

---

# Hardware

| Component | Purpose |
|-----------|----------|
| PYNQ-Z2 | FPGA Development Board |
| MQ-2 | Gas Detection |
| Flame Sensor | Fire Detection |
| HC-SR04 | Distance / Fall Detection |
| DHT11 | Temperature & Humidity |

---

# Software Stack

- Vivado 2023.1
- Verilog HDL
- Python
- PYNQ
- Jupyter Notebook
- PyTorch
- NumPy

---

# Repository Structure

```
DeepSafe/
│
├── rtl/                 # RTL modules
├── tb/                  # Testbenches
├── constraints/         # XDC constraints
├── scripts/             # Build scripts
├── weights/             # TinyML weights
├── notebooks/           # Dashboard
├── docs/
│   └── Architecture.md
├── images/
└── README.md
```

---

# FPGA Processing Pipeline

1. Acquire sensor data
2. Synchronize asynchronous inputs
3. Apply configurable moving average filters
4. Extract meaningful features
5. Run TinyML inference
6. Perform rule-based safety analysis
7. Fuse all decisions
8. Generate alerts
9. Display results on dashboard

---

# TinyML Accelerator

The project includes a lightweight fixed-point neural network implemented completely in FPGA fabric.

Architecture:

```
Input (4)

↓

Hidden (8)

↓

Hidden (4)

↓

Output (2)
```

Features:

- Q1.15 Fixed Point
- BRAM Weight Storage
- Hardware Inference
- AXI Configurable
- <10 µs processing latency

---

# Dashboard

The FPGA communicates with a PYNQ Jupyter Notebook which displays:

- Live sensor readings
- Alert status
- Confidence score
- Temperature
- Gas detection
- Flame detection
- Distance
- Event timestamps

---

# Current Status

- Sensor Interfaces
- RTL Development
- Hardware Filtering
- TinyML Integration
- AXI4-Lite Interface
- Dashboard Development
- Hardware Validation (In Progress)

---

# Future Improvements

- LoRaWAN Communication
- Cloud Synchronization
- Mobile Notification App
- Additional Gas Sensors
- Adaptive Learning
- Partial Reconfiguration

---

# Applications

- Manhole Worker Safety
- Underground Mining
- Sewage Monitoring
- Industrial Plants
- Chemical Storage
- Smart Infrastructure
- Tunnel Monitoring

---

# Project Highlights

- Edge AI Processing
- Fully Offline Operation
- FPGA Accelerated Analytics
- Multi-Sensor Fusion
- TinyML Inference Engine
- Real-Time Decision Making
- Safety-Critical Hardware Design

---

# Documentation

Detailed architecture, RTL module descriptions, implementation notes, and design decisions are available in:

```
docs/Architecture.md
```

---

## Team

**DeepSafe Development Team**

- Balaji K
- Madhavan R
- Nandavelan SPS
- Om Sharma M

**Institution:** Saveetha Engineering College  
**Department:** Electronics and Communication Engineering (ECE)  
**Platform:** PYNQ-Z2 (Xilinx Zynq-7020) 

---

# License

This project is released under the MIT License.

---

## Project Vision

> *"DeepSafe transforms conventional sensor monitoring into a reusable FPGA Edge Analytics IP Core capable of delivering real-time, intelligent safety decisions in environments where every microsecond matters."*
