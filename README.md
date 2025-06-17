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

## File Directory

**Bill of Materials**

├──BOM.xlsx

**Build Instructions**

├──....pdf

**User Guide**

├──___.pdf

├──tutorialVideo.mp4

**CAD Files**

├──CAD Files

│   └── FullAssembly.sldasm

│   └── FullAssembly.step

│   └── ComponentNamingTraker.xlsx

│   └── Part Files

│   └── Work Heads

Note: we have seperated the "work heads" (the parts which grab or directly interact with the muscles) since these parts are often modified for specific muscles being fabricated

**Software**

├──Source Code

├──HARP_GUI.exe

├──GUI_UserGuide.pdf

**Electronics**

├──Schematics.pdf
