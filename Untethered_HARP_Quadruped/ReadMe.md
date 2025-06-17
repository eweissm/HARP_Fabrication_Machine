# Open-Sourced HARP-Driven Untethered Quadruped

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

##  Directory Structure

```plaintext
├── BOM.xls
├── CAD Files/
│   └── LongLegQuadAssembly Final.STEP
├── Simulation/
│   └── QuadLegModel_5_1.m
├── Electronics/
│   └── Quadruped+Board.zip
