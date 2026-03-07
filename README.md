<h1 align="center"><b>Real-Time Flight Stabilization System (RT-FSS)</b></h1>

<p align="center">
  <b>FPGA-Based Hardware Pipeline for Real-Time Pitch Stabilization</b>
</p>

<p align="center">
<img src="https://github.com/user-attachments/assets/3f14d8d6-c4a4-42d8-a22a-ed1e2046cf7b" width="600">
</p>

---

## Overview

The **Real-Time Flight Stabilization System (RT-FSS)** is an FPGA-based control system designed to **maintain and correct the pitch orientation of a flight platform in real time**.

The system is implemented and tested on the **Xilinx Spartan-7 SP701 FPGA development board**.

When the platform tilts due to motion or external disturbances, RT-FSS continuously senses the motion, estimates the **pitch angle**, computes the corrective control output, and updates the actuator signal to restore stable pitch.

The complete stabilization path is implemented directly in **Verilog RTL**, forming a **deterministic real-time hardware pipeline** on the FPGA.

Instead of relying on a processor-based software loop, the system performs stabilization using **dedicated digital hardware**, enabling predictable timing and consistent control behavior.

---

## Real-Time Stabilization Loop

RT-FSS operates as a fixed-rate stabilization loop running **1000 times per second**.

Each loop cycle performs the full control sequence:

**read motion data → estimate pitch → compute correction → update actuator output**

This rapid update rate allows the system to detect disturbances quickly and apply corrective action to maintain stable pitch orientation.

---

## Purpose of the Project

The goal of RT-FSS is to demonstrate how **real-time flight stabilization can be implemented directly in FPGA hardware**.

The system forms a complete hardware feedback loop that:

- senses platform motion  
- estimates the pitch angle  
- computes corrective pitch control output  
- generates actuator PWM drive signals  

This architecture shows how **sensor-driven control systems can be implemented as deterministic hardware pipelines**.

---

## Motion Sensor Interface

RT-FSS uses the **MPU6050** motion sensor.

The sensor provides:

- **3-axis accelerometer data**
- **3-axis gyroscope data**

These measurements allow the system to determine how the platform is tilting and how the pitch angle is changing.

The sensor communicates with the FPGA through **I²C**, which uses two signal lines:

- **SCL** — clock line  
- **SDA** — data line  

Sensor data is acquired by the `mpu6050_reader` module and forwarded into the stabilization pipeline.

---

## System Flow

The stabilization pipeline operates through the following stages:

**Clock / Reset → Timing Control → Sensor Readout → Pitch Estimation → Control Computation → PWM Output**

In simple terms:

**Sense → Estimate Pitch → Correct Pitch → Actuate**

---

## Module Architecture

The RT-FSS stabilization system is implemented using modular RTL blocks.

Each module performs a specific role in the control pipeline.

---

### `reset_sync`

Synchronizes the reset signal with the FPGA clock.

**Purpose:**

- ensures reliable system startup  
- prevents metastability  
- initializes the system safely  

---

### `tick_gen`

Generates the timing base for the stabilization loop.

**Purpose:**

- controls when the control loop executes  
- maintains the **1000 updates per second** stabilization rate  
- synchronizes system operation  

---

### `mpu6050_reader`

Handles motion-data acquisition from the MPU6050 sensor.

**Purpose:**

- reads accelerometer and gyroscope values  
- performs sensor communication through **I²C**  
- supplies motion measurements to the control pipeline  

---

### `angle_filter`

Estimates the **pitch angle** from sensor measurements.

**Purpose:**

- processes motion data  
- produces a stable pitch estimate  
- provides the orientation input for correction  

---

### `pid_controller`

Computes the corrective output required to reduce pitch error.

**Purpose:**

- compares measured pitch with target pitch  
- calculates correction using PID control  
- generates actuator command  

---

### `pwm_servo`

Converts the control output into a PWM signal.

**Purpose:**

- generates actuator drive signal  
- updates the actuator command every control cycle  
- applies stabilization correction  

---

## Why FPGA Is Used

RT-FSS is implemented on an FPGA to achieve **deterministic real-time control performance**.

Using an FPGA provides:

- predictable timing  
- low control latency  
- parallel hardware execution  
- dedicated signal-processing pipeline  
- reliable real-time operation  

Instead of executing stabilization through sequential software instructions, the FPGA design behaves like a **dedicated hardware stabilization engine**.

---

## Why This Project Matters

Flight platforms require continuous stabilization to remain controlled during motion. Even small disturbances can cause pitch deviation, and delayed correction can reduce stability.

RT-FSS demonstrates how **fast and reliable pitch stabilization can be achieved using FPGA hardware**, where sensor acquisition, signal processing, control computation, and actuator generation operate together as a deterministic pipeline.

Such architectures are increasingly important in **modern aerospace systems, robotics, and autonomous platforms**, where reliable real-time control is essential for next-generation motion-stabilization technologies.

---

## Project Features

RT-FSS demonstrates:

- **Real-time pitch stabilization**
- hardware control loops running **1000 times per second**
- **sensor-driven closed-loop control**
- **FPGA-based deterministic control execution**
- **modular Verilog RTL system architecture**
- **complete sensor-to-actuator stabilization pipeline**
