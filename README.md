# Unmanned Aircraft Systems (UAS) Projects with MATLAB/Simulink

This repository contains a collection of MATLAB scripts and project work developed for the Unmanned Aircraft Systems (UAS) course (A.A. 2024-2025) at the University of Naples “Federico II.” The projects cover key aspects of UAS technology, including pose estimation, autopilot analysis, 3D path planning, and target tracking using an Extended Kalman Filter (EKF).

## Projects / Main Scripts

### 1. UAS Pose Estimation (`POSE_ESTIMATION.m`)

*   **Objective:** Estimate the pose (position and orientation) of a UAV during its landing phase using images captured by the drone's camera and AprilTag markers.
*   **Methodology:**
    *   Video Processing: Reads frames from `Agnano_Multiscale_Vertiport.avi` (`VideoReader`).
    *   AprilTag Detection: Utilizes `readAprilTag` (Computer Vision Toolbox) for various AprilTag families.
    *   Camera Intrinsics: Defines camera parameters using `cameraIntrinsics`.
    *   PnP Problem: Solves the Perspective-n-Point problem with `estimateWorldCameraPose`.
    *   Coordinate Transformation: Transforms orientation from camera to body frame (`angle2dcm`, `dcm2angle`).
    *   Error Analysis: Compares estimated pose with true trajectory data (`UAS_trajectory_24_25.mat`).
*   **Key MATLAB Functions:** `VideoReader`, `readAprilTag`, `cameraIntrinsics`, `estimateWorldCameraPose`, `angle2dcm`.

### 2. UAS Autopilot Analysis (Simulink-based)

*   **Objective:** Analyze and modify parameters within a pre-existing Simulink autopilot model (by Milone, Donnarumma & Norcaro) to study the impact on UAV flight behavior.
*   **Focus:** Investigated the effects of varying Proportional gain (`Kp`) and Damping ratio (`ζ`) in the Pitch Loop on UAV responsiveness and oscillation damping.
*   **Note:** This exercise involved working with a Simulink model (`Aerosonde_NON_linear` as per course material). The specific model file may not be included in this repository, but parameter modifications and result analysis were part of the project.

### 3. UAS 3D Path Planning (`RRTwithDubins_UAS2024.m`)

*   **Objective:** Plan a 3D flight path for a UAV in an environment with obstacles, using a Rapidly-exploring Random Tree (RRT) algorithm with Dubins path connections.
*   **Methodology:**
    *   Map Definition: Loads and modifies an occupancy map (`uavMapCityBlock.mat`, `updateOccupancy`).
    *   State Space & Dynamics: Defined using `ExampleHelperUAVStateSpace`.
    *   Path Planning: Employs `plannerRRT` and `validatorOccupancyMap3D`.
    *   Path Smoothing & Simulation: Uses `exampleHelperUAVPathSmoothing` and `exampleHelperSimulateUAV`.
    *   Wind Effects: Simulated by modifying `exampleHelperUAVDerivatives`.
*   **Key MATLAB Functions/Toolbox Elements:** `occupancyMap3D` functionalities, `ExampleHelperUAVStateSpace`, `plannerRRT`.

### 4. Target Tracking with Extended Kalman Filter (EKF) (`Tracking.m`)

*   **Objective:** Develop and implement an EKF to estimate the relative position of a target with respect to a UAV (EGO), using simulated radar measurements for collision avoidance.
*   **Methodology:**
    *   Data Handling: Loads simulation data (`SimulationData_2024_periodicalV_v2.mat`).
    *   Coordinate Transformations: Converts radar data from Spherical (SRF) to Cartesian (ENU).
    *   EKF Implementation:
        *   **State Model:** Constant velocity model for relative motion.
        *   **Matrices:** State Transition (`Phi`), Process Noise Covariance (`Q`), Measurement Covariance (`R`).
        *   **Initialization:** Uses first valid radar data and `RadEKFStateCov` (custom function).
        *   **Prediction & Correction Steps:** Standard EKF algorithm, using `RadEKFH` (custom function) for the measurement Jacobian.
    *   Performance Evaluation: Compares EKF estimates with true relative positions and analyzes standard deviations, including tuning of process noise (`qz`).
*   **Key MATLAB Functions:** `load`, `convang`, `RadEKFStateCov` (provided), `RadEKFH` (provided).

## Prerequisites

*   MATLAB (developed with a standard version)
*   **Required Toolboxes:**
    *   Computer Vision Toolbox (for `POSE_ESTIMATION.m`)
    *   Navigation Toolbox or similar (for `RRTwithDubins_UAS2024.m` functionalities like `ExampleHelperUAVStateSpace` and RRT)
    *   Simulink (for running the full autopilot model referenced in "UAS Autopilot Analysis")

## How to Run

For each `.m` script:
1.  Open the respective main script (e.g., `POSE_ESTIMATION.m`, `RRTwithDubins_UAS2024.m`, `Tracking.m`) in MATLAB.
2.  Ensure any required data files (e.g., `.mat` files, `.avi` video) are in the MATLAB path or the same directory.
3.  Review and adjust parameters within the script if desired.
4.  Run the script.
5.  Figures and outputs will be generated as described in the code and associated course material.

## Author

*   Antonio Carotenuto

## Acknowledgements

*   Prof. G. Fasano, for supervision, guidance, and providing base code/functions (e.g., `RadEKFStateCov`, `RadEKFH`, base RRT code) for the Unmanned Aircraft Systems course.
*   Milone, Donnarumma & Norcaro, for the Simulink autopilot model used as a basis for Exercise 2.
