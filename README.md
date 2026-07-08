# ⚙️ Planar Four-Bar Crank-Rocker Mechanism

[![Crank-Rocker Mechanism Demo](https://img.youtube.com/vi/C68PPp5x9YM/hqdefault.jpg)](https://www.youtube.com/shorts/C68PPp5x9YM)
*(Click the image above to watch the physical mechanism in action!)*

## 📖 Overview
This repository contains the computational MATLAB models and the final experimental report for the design, fabrication, and analysis of a planar four-bar crank-rocker mechanism. This project was conducted as part of the **ME 206: Statics and Dynamics** coursework at the **Indian Institute of Technology Gandhinagar (IITGN)**.

The objective of this project is to synthesize linkage dimensions that satisfy specific angular constraints, verify Grashof's Law, and perform a comparative analysis between theoretical kinematics (derived via vector loop equations) and experimental motion data extracted using Tracker video analysis.

## 📂 Repository Structure
* **`dimensional_synthesis.m`**: A symbolic MATLAB script that solves geometric constraint equations to determine the exact frame and coupler lengths required for the rocker to achieve a specific 17-degree angle at the halfway stage.
* **`kinematic_analysis.m`**: The primary computational model. It calculates the theoretical position (using the Diagonal Method) and the angular and linear velocities of the mechanism across its full rotation.
* **`ME206_Project_Report.pdf`**: The comprehensive project document detailing theoretical derivations, physical fabrication (MDF links with precision bearings), experimental setup, and analysis of numerical differentiation discrepancies.

## Mechanism Specifications & Grashof's Law
The mechanism was synthesized with the following axis-to-axis dimensions:
* **Ground Link (Frame, L1):** 13.5142 cm
* **Crank Link (Input, L2):** 10.0 cm
* **Coupler Link (L3):** 16.8117 cm
* **Rocker Link (Output, L4):** 20.0 cm

**Grashof's Verification:** 
For a continuous crank-rocker configuration, the sum of the shortest and longest links must be less than or equal to the sum of the remaining two links (`S + L <= P + Q`), and the shortest link must be grounded.
* `10.0 + 20.0 <= 16.8117 + 13.5142` 
* `30.0 <= 30.326` (Condition Satisfied)

## Kinematic Analysis Methodology
1. **Position Analysis:** Resolved analytically using the Diagonal Method to determine unknown coupler and rocker angles without numerical instability.
2. **Velocity Analysis:** Derived by differentiating the fundamental Vector Loop Closure equation with respect to time.
3. **Experimental Validation:** The physical prototype was driven by a constant 12V DC motor. 240fps video footage was processed to extract the real-world instantaneous linear velocity of the coupler's Center of Mass (COM), achieving error margins as low as 0.6% during stable acceleration phases.

## How to Run the Code
1. Clone this repository to your local machine.
2. Open **MATLAB**.
3. Run `dimensional_synthesis.m` to view the numerical resolution of the link lengths.
4. Run `kinematic_analysis.m` to compute the velocities. You can manually adjust the `theta2_deg` variable in the script to analyze the kinematics at specific crank angles.

---
*Project Team: Rishabh Shah, Saujas Seth, Sejal Sharbidre, and Shah Dhvani*
