# EPFL ME-425 Fall 2024 Final Project: Model Predictive Control Car Simulation

## 📋 Overview

This repository contains the implementation of a Model Predictive Control (MPC) framework for an autonomous car simulation in MATLAB. Developed as the final project for **ME-425: Model Predictive Control** at EPFL under Prof. Collin Jones, the controller handles lateral and longitudinal tasks such as lane-keeping, velocity tracking, tube-based safety, and overtaking maneuvers.

> **Imagine** your car on the highway smoothly changing lanes or keeping a steady speed without you lifting a finger—this project brings that vision to life in simulation.

## 📑 Key Deliverables

1. **Analytical Linearization** of the car model around steady cruise.
2. **Subsystem Decomposition** into independent lateral and longitudinal dynamics.
3. **Linear MPC Controllers** with terminal sets for both subsystems.
4. **Offset-Free Tracking** via disturbance estimation.
5. **Tube-Based MPC** ensuring safety in car-following scenarios.
6. **Nonlinear MPC (NMPC)** for complex maneuvers and overtaking with ellipsoidal constraints.

## 📊 Results and Visualization

* Lane change in ≤3 s without overshoot.
* Speed settling within 10 s from 80 km/h to 120 km/h.
* Safe minimum distance ≥ 8 m in tube MPC.
* Smooth overtaking with passenger-comfort constraints.

## ✍️ Authors

* **Yo-Shiun Cheng** (386249)
* **Jiwon You** (394716)

**Supervisor:** Prof. Collin Jones, EPFL


## 🙏 Acknowledgments

Thanks to the ME-425 teaching team for guidance and the MPT3 developers for polytope utilities.
