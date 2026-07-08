<div align="center">

# 🔩 Four-Bar Crank-Rocker Mechanism — Design, Synthesis & Kinematic Validation

**ME 206: Statics and Dynamics — Experiment 4**
*Indian Institute of Technology Gandhinagar*

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023%2B-orange?logo=mathworks&logoColor=white)](https://www.mathworks.com/products/matlab.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](#license)

</div>

---

## Overview

This repository documents the complete design-to-validation pipeline for a **planar four-bar crank-rocker mechanism**, developed as part of the ME 206 (Statics and Dynamics) coursework at IIT Gandhinagar. The objective was to design a mechanism in which the input crank completes a full 360° rotation while the output rocker oscillates between fixed limits, and to rigorously predict and verify the kinematic behavior of its coupler link.

The project combines **analytical dimensional synthesis** (deriving link lengths that satisfy a target rocker output angle at a prescribed configuration), **MATLAB-based kinematic modeling** (position and velocity analysis via the vector-loop / diagonal method), and **physical fabrication and experimental validation** using video-tracking software. The mechanism was laser-cut from MDF, driven by a 12V DC motor, and instrumented with fiducial markers at the crank pin, coupler pin, and coupler center of mass for motion capture.

The core engineering question addressed here is: *how accurately can closed-form analytical kinematics predict the real-world motion of a physically fabricated linkage, and what are the dominant sources of error when comparing theory to experiment?* This repository walks through that entire process — from Grashof-law-compliant synthesis, to MATLAB implementation, to Tracker-based motion capture, to a full error analysis of the discrepancies observed.

---

## Key Features

- Planar four-bar **crank-rocker** mechanism satisfying Grashof's Law
- Closed-form **dimensional synthesis** for a prescribed rocker angle at a target configuration
- Analytical **position analysis** using the diagonal (triangle-decomposition) method
- Analytical **velocity analysis** for coupler and rocker angular velocities
- **Coupler center-of-mass velocity** derivation and computation
- Two standalone **MATLAB scripts** — synthesis and kinematic analysis
- Physical **fabrication** with laser-cut MDF links, precision ball bearings, and a DC motor
- **Tracker**-based video motion capture and digitization
- Full **analytical vs. experimental** comparison across a 360° crank cycle
- Detailed **error-source analysis** (phase sync, zero-crossing singularity, differentiation noise)

---

## Demonstration

<div align="center">

[![Watch the mechanism in action](https://img.youtube.com/vi/C68PPp5x9YM/0.jpg)](https://www.youtube.com/shorts/C68PPp5x9YM)

**[Watch the fabricated mechanism in motion](https://www.youtube.com/shorts/C68PPp5x9YM)**

</div>

---

## Project Workflow

```
Design Requirements
       │
       ▼
Dimensional Synthesis (Mathematica / MATLAB)
       │
       ▼
Mechanism Fabrication (Laser-cut MDF + DC Motor + Bearings)
       │
       ▼
MATLAB Kinematic Analysis (Position → Velocity → COM Velocity)
       │
       ▼
Experimental Video Capture (Smartphone Camera)
       │
       ▼
Tracker Motion Digitization
       │
       ▼
Comparison with Analytical Theory
       │
       ▼
Error Analysis & Discussion
```

---

## 🗂️ Repository Structure

```
Project/
│
├── README.md                     # This file
├── ME206_Project_Report.pdf      # Full project report
├── dimensional_synthesis.m       # MATLAB: link-length synthesis (symbolic solve)
├── kinematic_analysis.m          # MATLAB: position/velocity kinematic analysis
├── videos/                       # Working model video
```
---

## Theory

### Four-Bar Linkage
A four-bar linkage consists of four rigid links connected by revolute (pin) joints, forming a single closed kinematic chain with one degree of freedom. It is one of the most fundamental mechanisms in machine design, capable of producing a wide variety of coupler-point trajectories from a single rotary input.

### Crank-Rocker Configuration
In a crank-rocker mechanism, the shortest link (grounded to the input) rotates fully through 360°, while the link opposite the ground (the rocker) oscillates back and forth within a bounded angular range. This behavior is only guaranteed when the link lengths satisfy **Grashof's Law** and the shortest link is grounded adjacent to the fixed frame.

### Grashof's Law
For links classified as Shortest (S), Longest (L), and the two remaining Intermediate links (P, Q):

$$S + L \leq P + Q$$

For this design:

| Link | Description | Length (cm) |
|---|---|---|
| r₁ | Ground / Frame | 13.5142 |
| r₂ | Crank (Shortest, Input) | 10.0 |
| r₃ | Coupler | 16.8117 |
| r₄ | Rocker (Output) | 20.0 (Longest) |

**Check:** `10 + 20 ≤ 16.8117 + 13.5142` → `30 ≤ 30.326` ✅ — condition satisfied, confirming valid crank-rocker behavior.

### Dimensional Synthesis
Rather than choosing link lengths arbitrarily, the frame and coupler lengths were derived analytically to satisfy a **precision-point constraint**: the rocker must reach a specific output angle (17°) at a defined "halfway stage" configuration, given a fixed crank length (10 cm) and rocker length (20 cm). This was solved as a system of nonlinear geometric constraint equations (originally in Mathematica, replicated in MATLAB via symbolic solving), yielding the ground link length and coupler length used in fabrication.

### Position Analysis — The Diagonal Method
The quadrilateral formed by the four links is decomposed into two triangles using a diagonal connecting the crank pin (B) to the rocker pivot (D). The Law of Cosines is applied twice — once per triangle — to solve for the unknown coupler angle (θ₃) and rocker angle (θ₄) at any given crank angle (θ₂), without needing iterative numerical solvers.

### Velocity Analysis
Differentiating the vector-loop closure equation with respect to time yields a system of two linear equations in the angular velocities ω₃ and ω₄, which can be solved analytically in closed form once θ₂, θ₃, θ₄, and ω₂ are known.

### Coupler Center-of-Mass Velocity
The absolute velocity of the coupler's center of mass (G₃) is obtained as the vector sum of the velocity of the crank-pin (A) and the relative velocity of G₃ with respect to A, resolved into Cartesian components.

---

## Mathematical Formulation

**Diagonal length (Law of Cosines, triangle O₂–A–O₄):**

$$l_{BD} = \sqrt{r_1^2 + r_2^2 - 2 r_1 r_2 \cos\theta_2}$$

**Intermediate triangle angles:**

$$\beta = \cos^{-1}\left(\frac{r_1^2 + l_{BD}^2 - r_2^2}{2 r_1 l_{BD}}\right), \qquad \phi = \cos^{-1}\left(\frac{r_4^2 + l_{BD}^2 - r_3^2}{2 r_4 l_{BD}}\right)$$

**Rocker and coupler angles (open configuration):**

$$\theta_4 = 180^\circ - (\beta + \phi), \qquad \theta_3 = \sin^{-1}\left(\frac{r_4 \sin\theta_4 - r_2 \sin\theta_2}{r_3}\right)$$

**Angular velocities of coupler and rocker:**

$$\omega_3 = \frac{r_2 \,\omega_2 \sin(\theta_4 - \theta_2)}{r_3 \sin(\theta_3 - \theta_4)}, \qquad \omega_4 = \frac{r_2 \,\omega_2 \sin(\theta_3 - \theta_2)}{r_4 \sin(\theta_3 - \theta_4)}$$

**Coupler COM velocity:**

$$\vec{v}_{G_3} = \vec{v}_A + \vec{v}_{G_3/A}, \qquad v_{G_3} = \sqrt{v_{G_{3x}}^2 + v_{G_{3y}}^2}$$

**Sign convention:** negative ω values indicate clockwise rotation; positive values indicate counter-clockwise rotation (right-hand rule, z-axis out of the page).

---

## MATLAB Implementation

### 1. `dimensional_synthesis.m`

**Purpose:** Solves the geometric synthesis problem to determine the required frame length (`b`) and coupler length (`lprime`) that make the rocker reach a target angle at a specified "halfway stage" configuration.

| | |
|---|---|
| **Inputs** | `r = 10` (crank length), `l = 20` (rocker length), `theta = 17°` (target rocker angle) |
| **Outputs** | `b` (frame/ground length), `lprime` (coupler length), `beta` (auxiliary angle) |
| **Key MATLAB functions** | `syms`, `vpasolve`, `deg2rad`, `char` |

**Algorithm:**
1. Define `b`, `lprime`, `beta` as symbolic unknowns.
2. Formulate three geometric constraint equations relating the crank, rocker, frame, and coupler at the target configuration.
3. Solve the nonlinear system numerically with `vpasolve`.
4. Print the resulting frame and coupler lengths.

**Equations implemented:**

$$b^2 = l'^2 - (l-r)^2 \qquad l\cos\theta + l'\cos\beta = b \qquad l\sin\theta + r = l'\sin\beta$$

**Expected output:** `b ≈ 13.5142 cm`, `lprime ≈ 16.8117 cm` — the exact link lengths used in fabrication.

**Computational complexity:** Negligible — a single symbolic solve of 3 nonlinear equations in 3 unknowns; runs in well under a second.

---

### 2. `kinematic_analysis.m`

**Purpose:** Given the finalized link lengths, computes full position, velocity, and coupler COM kinematics for a specified crank angle.

| | |
|---|---|
| **Inputs** | `L1, L2, L3, L4` (link lengths), `theta2_deg` (crank angle), `omega2` (crank angular velocity), `r_G3` (distance from crank pin to coupler COM) |
| **Outputs** | `theta3`, `theta4` (position), `omega3`, `omega4` (angular velocities), `v_G3x`, `v_G3y`, `v_G3_mag` (coupler COM velocity) |
| **Key MATLAB functions** | `deg2rad`, `rad2deg`, `acos`, `asin`, `sqrt`, `isreal`, `fprintf` |

**Algorithm (4 modules):**
1. **Initialization** — define link lengths, convert input crank angle to radians, set offset angle.
2. **Position Analysis** — compute diagonal length `l_BD`, intermediate angles `beta_val`/`psi_val`, then solve for `theta4` (open configuration) and `theta3` via the sine rule, with a validity check (`isreal`) to catch non-assemblable configurations.
3. **Velocity Analysis** — solve the linearized vector-loop equations analytically for `omega3` and `omega4`.
4. **COM Kinematics** — compute crank-pin velocity components, then resolve the coupler COM velocity and its magnitude.

**Expected output (for θ₂ = 30°, ω₂ = -4.3387 rad/s):** printed diagonal length, θ₃, θ₄, ω₃, ω₄, and coupler COM velocity components/magnitude to the console.

**Computational complexity:** O(1) per crank angle — purely closed-form trigonometric evaluation, no iterative solvers; trivially extendable to a full 360° sweep via a loop.

---

## Experimental Setup

- **Fabrication:** Links laser-cut from MDF sheets to precise axis-to-axis dimensions.
- **Actuation:** 12V DC motor driving the crank at (approximately) constant angular velocity.
- **Bearings:** Precision ball bearings (ID: 5 mm, OD: 1.6 cm) at all pivot joints to minimize friction; aluminum rod shafts passed through the inner race for smooth relative rotation.
- **Markers:** Fiducial markers placed at pivot joints A and B, and at the coupler center of mass (G₃), for video tracking.
- **Video Capture:** Motion recorded with a camera positioned parallel to the plane of motion to eliminate parallax.
- **Tracker Software:** [Tracker](https://physlets.org/tracker/) used to digitize marker trajectories frame-by-frame.
- **Calibration:** A 13.5 cm reference length (the ground link) was used for spatial calibration; the coordinate origin was fixed at the crank pivot (O₂), with the X-axis aligned to the ground link.
- **Derived Quantity:** Coupler COM offset `r_G3 ≈ 6.4 cm`, measured from Tracker data.
- **Input Angular Velocity:** `ω₂,avg = -4.3387 rad/s`, obtained via numerical differentiation (`ω = dϕ/dt`) of the tracked crank angle time-series.
- **Data Processing:** Position files synchronized on a common time grid; a least-squares circle fit located the fixed pivot O₂; ω₃ and v_G3 were computed via central-difference numerical differentiation.

---

## Results

**Design validation:** Grashof's Law was satisfied (30 ≤ 30.326), and the synthesized dimensions physically achieved the targeted 17° rocker angle at the halfway-stage configuration, confirming successful synthesis and fabrication.

**Analytical vs. Experimental comparison** across one full crank cycle (ω₂ = -4.3387 rad/s, r_G3 = 6.4 cm):

| Crank Angle θ₂ [deg] | ω₃ Analytical [rad/s] | ω₃ Exp. [rad/s] | ω₃ % Error | v_G3 Analytical [m/s] | v_G3 Exp. [m/s] | v_G3 % Error |
|---|---|---|---|---|---|---|
| 0 | 1.30 | 2.85 | 119.2% | 0.362 | 0.440 | 21.6% |
| 30 | 5.99 | 3.97 | 33.8% | 0.225 | 0.224 | **0.6%** |
| 60 | 1.29 | 4.53 | 252.6% | 0.358 | 0.311 | 13.0% |
| 90 | 0.00 | 0.88 | – | 0.434 | 0.420 | **3.2%** |
| 120 | -0.69 | -0.18 | 74.4% | 0.443 | 0.429 | **3.2%** |
| 150 | -1.27 | -0.92 | 27.4% | 0.421 | 0.417 | **1.0%** |
| 180 | -1.85 | -1.51 | 18.3% | 0.382 | 0.362 | 5.1% |
| 210 | -4.94 | -2.40 | 51.3% | 0.389 | 0.286 | 26.4% |
| 240 | -6.79 | -3.13 | 54.0% | 0.225 | 0.260 | 15.8% |
| 270 | 0.00 | -2.34 | – | 0.434 | 0.168 | 61.3% |
| 300 | -8.08 | -3.46 | 57.1% | 0.259 | 0.252 | **2.7%** |
| 330 | -14.91 | -4.14 | 72.2% | 0.828 | 0.472 | 43.0% |

**Key observations:**
- The coupler COM trajectory forms a smooth, continuous closed loop, confirming the linkage remained intact and binding-free throughout the cycle.
- At stable-velocity phases (e.g., 30°, 90°, 150°, 300°), errors drop as low as **0.6%–3.2%**, indicating high tracking accuracy under favorable conditions.
- The largest errors occur near velocity extrema and zero-crossings, where small timing misalignments produce disproportionately large percentage deviations.

---

## Error Analysis

- **Phase Synchronization Error (primary source):** Experimental `t = 0` must align precisely with analytical `θ₂ = 0`. Even a small timing offset compares experimental and analytical velocities at *different* crank angles — devastating in regions of high angular acceleration.
- **Zero-Crossing Singularity:** Near θ₂ = 90° and 270°, the analytical ω₃ passes through zero. Since percentage error is normalized by the analytical value, this produces mathematically undefined or extremely inflated errors even when absolute deviation is small.
- **Differentiation Noise:** Velocity is obtained by numerically differentiating tracked pixel positions, which acts as a high-pass filter — amplifying small pixel-level jitter into visible high-frequency velocity fluctuations, even though the true motion is smooth.
- **Net Assessment:** These are measurement/analysis artifacts rather than evidence of mechanical failure — the trajectory and low-error regions confirm the mechanism performs as designed.

---

## Applications

- Educational demonstration of vector-loop kinematic synthesis and analysis
- Prototyping platform for planar linkage-based automation mechanisms
- Benchmark case study for **video-based (markerless/marker) motion capture** validation
- Foundation for oscillating mechanisms in packaging, textile, and pick-and-place machinery
- Teaching tool for Grashof's Law and crank-rocker classification

---

## Future Improvements

- **Direct Measurement Sensors:** Mount an IMU on the coupler to obtain direct angular velocity/acceleration, bypassing differentiation noise entirely.
- **Rotary Encoders:** Add a rotary encoder on the crank to measure θ₂ precisely in real time, solving the phase-synchronization problem by plotting velocity vs. angle instead of vs. time.
- **PID Motor Control:** Implement closed-loop PID control to maintain a strictly constant ω₂, compensating for variable gravitational torque on the linkage.
- **Material Upgrade:** Replace MDF links with aluminum or acrylic to reduce out-of-plane vibration/bending and improve marker-tracking fidelity.

---

## Installation

1. Install [MATLAB](https://www.mathworks.com/products/matlab.html) (R2020b or later recommended).
2. Clone this repository:
   ```bash
   git clone https://github.com/TODO-username/TODO-repo.git
   ```
3. Open the project folder in MATLAB.
4. Run the synthesis script to derive/verify link lengths:
   ```matlab
   dimensional_synthesis
   ```
5. Run the kinematic analysis script to compute position/velocity results:
   ```matlab
   kinematic_analysis
   ```

---

## Requirements

- MATLAB (base installation)
- MATLAB **Symbolic Math Toolbox** (required for `dimensional_synthesis.m`)
- [Tracker Video Analysis Software](https://physlets.org/tracker/) (for reproducing experimental digitization — optional, not required to run the MATLAB scripts)

---

## Screenshots

> Populate these sections with actual images from `images/` as they become available.

### Mechanism
<img width="270" height="293" alt="97e282c3-8349-48e8-aa70-4f13a17cdd68" src="https://github.com/user-attachments/assets/584ca61c-f027-4062-bd98-6e8691f7f527" />

### Experimental Setup
<img width="314" height="226" alt="image" src="https://github.com/user-attachments/assets/03abc8f6-f7b3-4913-b65b-1e69a85ac514" />

### Graphs
<img width="359" height="287" alt="image" src="https://github.com/user-attachments/assets/7dd9d6ae-892b-442d-a9b0-6b824431213d" />
<img width="355" height="302" alt="image" src="https://github.com/user-attachments/assets/d6ae31ea-dd83-4d75-90e1-b5f200fb5f3b" />


---

## References

1. Hibbeler, R. C. (2018). *Engineering Mechanics: Dynamics*. Pearson.
2. Tracker Video Analysis and Modeling Tool — [https://physlets.org/tracker/](https://physlets.org/tracker/)
3. Jayaprakash, K. R. *ME206: Lectures on Statics and Dynamics*, IIT Gandhinagar, Fall 2025.
4. Beer, F. P., Johnston, E. R. *Vector Mechanics for Engineers: Dynamics*, 12th Edition, McGraw-Hill Education.

---

## Acknowledgements

We express our sincere gratitude to **Prof. K. R. Jayaprakash** and the Teaching Assistants of ME206 for their guidance throughout this project. We also acknowledge the **Tracker Development Team** for their open-source software, which was integral to the experimental analysis.

---

## Authors

| Name | Roll No. |
|---|---|
| Shah Rishabh | 24110327 |
| Sejal Sharbidre | 24110322 |
| Dhvani Shah | 24110325 |
| Saujas Seth | 24110319 |

*Group 15 — ME 206: Statics and Dynamics, IIT Gandhinagar*

---

## License

This project is licensed under the **MIT License**.

---
