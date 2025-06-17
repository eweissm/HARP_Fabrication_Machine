# HARP Controller

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

