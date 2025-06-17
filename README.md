# HARP Replication Reources
Here we present all necessary resources for replication of the HARP, barring what is already present in the manuscript and supplemental materials.

Enclosed in this repsitory is:

1.  The open sources fabrication platform
2.  CAD files for the geometric anisotropy HARP
3.  Easy fabrication instructions and a classroom guide
4.  All codes related to characterization and position control on the isotonic test stand
5.  The continuum robot controllers
6.  All design files and codes for the untethered quadruped

## File Directory

```plaintext
├── Open Sourced Fabrication Platform
│   └──___.pdf
│   └──tutorialVideo.mp4
│   └──CAD Files
│   │   └── FullAssembly.sldasm
│   │   └── FullAssembly.step
│   │   └── ComponentNamingTraker.xlsx
│   │   └── Part Files
│   │   └── Work Heads
│   ├──Source Code
│   ├──HARP_GUI.exe
│   ├──GUI_UserGuide.pdf
│   ├──Electric_Schematics.pdf
├── GeometricHARP_CADFiles
├── Easy Fabrication Resources
│   └── BOM.xlxs
│   └── HARPIntructions.pdf
├── Untethered_HARP_Quadruped
│   └── BOM.xls
│   └──CAD Files/
│   │   └──LongLegQuadAssembly Final.STEP
│   └── Simulation/
│   │   └── QuadLegModel_5_1.m
│   └── Electronics/
│   │   └── Quadruped+Board.zip
├── HarpController
├── HARP Continuum Robot Control
│   └── ContinuumRobotArduino.ino
│   └── ContinuumRobotControl_Modelless.py
│   └── ContinuumRobotControl_Modelless_BODE.py
│   └── ContinuumRobotControl_ConstantCurvature.py
│   └── ContinuumRobotControl_ConstantCurvature_BODE.py
│   └── SerialCommunicator.py
│   └── NatNetClient.py
│   └── DataDescriptions.py
│   └── MoCapData.py
│   └── RobotLineArt/
│   │     └── ASUlogo.svg
│   └── RobotData/
│         └── robot_log_*.csv
```



# Open Sourced HARP Fabrication Machine

## Introduction
Note: we have seperated the "work heads" (the parts which grab or directly interact with the muscles) since these parts are often modified for specific muscles being fabricated


# GeometricHARP_CADFiles
Presented here are all the CAD files for the HARPs which derive their anisotropy from geometry alone. 

### The ribbed tube uses 2 files:

|--RibbedTube_mold.sldprt

|--RibbedTube_mold_top.sldprt

A 2 mm brass rod is used as the mandrel


### The 4 point star uses 3 files:

|--StarTube_mold_bottom_5_21_4POINT.sldprt

|--StarTube_mold_TOP_5_21_4POINT.sldprt

|--StarTube_mold_TOP_5_21_4POINT_SLA.sldprt



### The 5 point star uses 3 files:

|--StarTube_mold_bottom_5_21.sldprt

|--StarTube_mold_TOP_5_21.sldprt

|--StarTube_mold_TOP_5_21.sldprt



### The 4 point star uses 3 files:

|--StarTube_mold_bottom_5_21_6POINT.sldprt

|--StarTube_mold_TOP_5_21_6POINT.sldprt

|--StarTube_mold_TOP_5_21_6POINT.sldprt


## Print Settings

- All parts were printed on a Prusa MK4 with AmazonBasic PLA. 
- Layer height was 0.15mm.
- Infill density = 25%
- Support on, Snug
- 2 raft layers used

  
# Easy Fabrication Resources

## Aim
Here we provide detailed instructions and materials for getting young learners interested in soft robotics. We recommend this project for children of at least 10 years old, who may need adult assistance with operating an oven and using a scissor.

## Instructions
See HARPInstructions.pdf

## Required Materials
See BOM.xls
- Oven (capable of 150 C)
- Silicone Tube (durometer 50, 5/32x3/32”)
- Nylon Fishing Line (.038” (“The Core”)
- Nylon Fishing Line (.017”) (“The fiber”)
- Duct Tape
- Copper Wire (26 AWG)
- Mandrel (4x300mm aluminum rod)
- Leur Lock Barbed Connector (1/8”)
- Barbed plug
- Syringe (10ml)


# Untethered_HARP_Quadruped
**Author:** Eric Weissman  
**Last Updated:** May 2025

This repository contains all design files, simulation tools, and documentation needed to build and test an untethered quadruped robot actuated by HARP-driven pneumatic legs.

---

##  Bill of Materials (BOM)

- A complete list of components and part numbers is provided in:  
  **`BOM.xls`**

---

## 🛠 CAD Files

- Full mechanical assembly in STEP format:  
  **`CAD Files/LongLegQuadAssembly Final.STEP`**

- Use any CAD software that supports `.STEP` files (e.g., SolidWorks, Fusion 360, FreeCAD) to view or modify.

---

##  Simulation

- MATLAB simulation model of a single leg is provided in:  
  **`Simulation/QuadLegModel_5_1.m`**

- This model uses **Peter Corke's Robotics Toolbox** for MATLAB.  
  Please install it before running the simulation:  
  [Peter Corke's Robotics Toolbox](https://petercorke.com/toolboxes/robotics-toolbox/)

---

##  Electronics

- Custom controller PCB design is provided in Eagle format:  
  **`Electronics/Quadruped+Board.zip`**

- Includes:
  - `.brd` and `.sch` Eagle design files
  - Part placements and annotations

- Compatible with Autodesk Eagle and Fusion 360 Electronics.

---

# HarpController

**Author:** Eric Weissman  
**Last Updated:** May 2025

This codebase provides tools for both **open-loop** and **closed-loop** control of a pneumatic continuum actuator known as the HARP (Hydraulically Actuated Robotic Pneumatic system). It includes scripts for real-time pressure control, Bode analysis, and isotonic actuation experiments, all integrated with an Arduino Nano and pressure regulators.

## Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Control Modes](#control-modes)
  - [Closed-Loop Control](#closed-loop-control)
  - [Open-Loop Control](#open-loop-control)
  - [Bode Analysis](#bode-analysis)
  - [Isotonic Tests](#isotonic-tests)
- [Dependencies](#dependencies)
- [Usage Notes](#usage-notes)
- [License](#license)

---

## Overview

An Arduino Nano (running `HARP_Controller.ino`) receives pressure setpoints over serial communication from a host computer. It then modulates a pressure regulator accordingly.

Four main control modes are implemented:

1. **Closed-loop control** using a PID controller and muscle model.
2. **Open-loop control** with a periodic pressure waveform.
3. **Bode Analysis**
4. **Isotonic Characterization**

---

## Project Structure

| File / Directory             | Description                                                                 |
|-----------------------------|-----------------------------------------------------------------------------|
| `MuscleClass.py`            | Defines the `Muscle` class for simulating behavior and generating lookups. |
| `serial_communicator.py`    | Handles buffered serial I/O between PC and Arduino.                        |
| `MuscleConfig.ini`          | Configuration file for muscle parameters.                                  |
| `ControlledSinusoid.py`     | Main script for **closed-loop and Open-loop sinusoidal tracking**.                       |
| `SinusoidV2.py`             | Script for **isotonic tests**.                                   |
| `BodeAnalysis.py`           | Script for **logging Bode analysis response**.                             |
| `Data/`, `Data_final/`      | Output folders for CSV logs.                                               |

---

## Control Modes

### Closed-Loop Control

- **Python Script:** `ControlledSinusoid.py`  
- **Arduino Sketch:** `HARP_Controller.ino`

This script performs real-time PID tracking of a 3 Hz sinusoidal trajectory. It uses a physics-based model to estimate required muscle pressure from desired strain.

**Key Features:**

- Real-time bidirectional serial communication
- Dynamic pressure estimation via lookup table
- PID control (tuned for 3 Hz)
- Safety-capped pressures (0–50 psi)
- Buffered logging to CSV

**Adjustable Parameters:**

```python
output_file     # Path for saving logged data
BATCH_SIZE      # Entries before logging to file
ReadFrequency   # Sampling rate in Hz
maxP            # Max pressure (psi)
ConfigFile      # Path to muscle configuration
```

---

## Open-Loop Control

- **Python Script:** `SinusoidV2.py`  
- **Arduino Sketch:** `HARP_Controller.ino`

This script applies a periodic sawtooth pressure waveform to the muscle in an open-loop manner. It is suitable for fatigue and isotonic testing, where external feedback is not used for pressure control.

**Waveform Profile:**

- Ramps pressure up from 0 to `maxP` over `rampTime` seconds
- Holds at `maxP` for `edgeTime` seconds
- Ramps pressure down back to 0 over `rampTime` seconds
- Holds at 0 for `edgeTime` seconds
- Repeats the above cycle continuously

**Adjustable Parameters:**

```python
rampTime = 15       # Seconds for pressure ramp up/down
edgeTime = 3        # Seconds to hold pressure at max and min
maxP = 40           # Max pressure in psi
freq = 100          # Loop update frequency in Hz
```
---

## Bode Analysis

- **Python Script:** `BodeAnalysis.py`  
- **Arduino Sketch:** `BodeAnalysis.ino`

This mode is used to evaluate the dynamic frequency response of the muscle system. It logs time-series data from the Arduino while sinusoidal pressure inputs are applied at varying frequencies. The data can then be used to construct a Bode plot for system identification.

### Features

- One-way high-frequency data logging from Arduino to PC
- Records multiple sensor values per timestamp
- Automatically buffers and saves to CSV

### Configuration Parameters

```python
direction = 'oneWayFromArduino'
n_floats_from_arduino = 6
n_floats_to_arduino = 6
output_file = "Data_final/BodeAnalysis_<experiment_name>.csv"
```

---

## Usage Notes

- Ensure the `output_file` does **not already exist** — scripts will raise a `FileExistsError` to prevent accidental data loss.
- The default serial port is set to `"COM4"`. Update this to match the port your Arduino is connected to.
- Sampling rates (`freq`, `ReadFrequency`) and logging frequency (`logFreq`) can be adjusted in each script for performance tuning.
- The Arduino must be running the appropriate `.ino` sketch (`HARP_Controller.ino` or `BodeAnalysis.ino`) before running any Python script.
- Serial communication is handled by `SerialCommunicator`, which buffers data and supports both one-way and two-way modes.
- Pressure safety limits are enforced in all control scripts to prevent over-pressurization.
- Muscle rest length (`L0`) is calibrated during the first few seconds of the closed-loop control script.




# HARP Continuum Robot Control

**Author:** Eric Weissman  
**Last Updated:** May 2025

This repository contains the full control stack for a soft continuum robot system (HARP) actuated using pneumatic artificial muscles. It includes Arduino firmware and multiple Python control scripts for modelless and model-based control approaches, using OptiTrack motion capture for feedback.

---

## System Overview

- **Robot:** Soft continuum robot (HARP)
- **Actuation:** Pneumatic muscle bundles
- **Sensors:** Motion capture via OptiTrack (NatNet SDK)
- **Control Loop Rate:** 100–200 Hz
- **Pressure Regulation:** Arduino + PWM control
- **Control Methods:**
  - **Modelless PID:** Jacobian-based Cartesian control
  - **Constant Curvature:** Physics-based curvature estimation
- **Interface:** Real-time visualization via PyQtGraph
- **Data Logging:** CSV export of pose, target, error, and pressures

---

## Main Components

### `ContinuumRobotArduino.ino`

**Platform:** Arduino  
**Role:** Low-level actuator interface

- Reads 3–9 pressure setpoints from serial (float values with sync byte `0xAA`)
- Converts pressure values to PWM voltages to drive analog regulators
- Reads actual pressures via analog sensors and sends back float values
- Max pressure constrained to my maxP variable


---

### `ContinuumRobotControl_Modelless.py`

**Approach:** Modelless Cartesian PID control

- Tracks robot tip pose in X-Z plane using PID
- Uses Jacobian-based pressure mapping via `scipy.optimize.lsq_linear`
- Real-time visualization with PyQtGraph
- GUI for live PID gain tuning
- Tracks arbitrary SVG-defined paths or sine wave trajectories
- Logs control and performance data to CSV

---

### `ContinuumRobotControl_Modelless_BODE.py`

**Approach:** Same as above + frequency analysis

- Adds BODE plot generation and signal injection tools
- Useful for identifying system response characteristics

---

### `ContinuumRobotControl_ConstantCurvature.py`

**Approach:** Physics-based constant curvature control

- Computes curvature (κ) and angle (ϕ) from Cartesian pose
- Uses constant curvature kinematics and inverse muscle model
- Computes bending moments using Euler-Bernoulli beam theory
- Solves for muscle pressures using `lsq_linear`
- PID feedback applied to curvature and angle errors
- GUI for PID tuning + trajectory tracking

---

### `ContinuumRobotControl_ConstantCurvature_BODE.py`

**Approach:** Constant curvature + system ID

- Adds experimental tools for step response and BODE characterization
- Useful for validating the beam bending model and controller tuning

---

## Dependencies

**Python Packages:**
- `numpy`
- `scipy`
- `pyqtgraph`
- `PyQt5`
- `matplotlib`
- `svgpathtools`

**Custom Modules (must be in Python path):**
- `SerialCommunicator.py`
- `NatNetClient.py`
- `MoCapData.py`
- `DataDescriptions.py`

---

##  Usage

1. Upload `ContinuumRobotArduino.ino` to your Arduino board.
2. Connect the Arduino to your computer (`port="COM13"` by default).
3. Ensure OptiTrack Motive is running and streaming rigid body data.
4. Choose and run a Python control script:
    - `Modelless.py` for Cartesian PID
    - `ConstantCurvature.py` for physics-based control
5. Adjust PID gains via the GUI if needed.
6. Observe real-time pose tracking and pressure display.
7. Use `Ctrl+C` to safely stop the program and flush logs.

---

## Notes
* Live trajectory tracking uses SVG paths; modify or replace ASUlogo.svg as needed.

* Model-based control assumes consistent backbone stiffness and muscle placement.

* For experimental dynamics, use BODE variants to assess frequency response.


