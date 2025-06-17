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

# HARP Continuum Robot Control
