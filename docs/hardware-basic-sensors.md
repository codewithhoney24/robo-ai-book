---
title: "Hardware: Basic Sensors – Cameras, LiDAR, aur IMUs kaam kaise karte hain (bagair coding ke)"
sidebar_label: "Hardware: Basic Sensors"
description: "Understanding basic robot sensors including cameras, LiDAR, and IMUs without coding"
---

# Hardware: Basic Sensors

## Introduction

In this module, we'll explore the fundamental sensors used in robotics without diving into coding. Understanding these sensors is crucial for building effective robotic systems.

## Cameras

Cameras are essential for visual perception in robotics:

- **RGB Cameras**: Capture color images similar to human vision
- **Stereo Cameras**: Provide depth information by comparing images from two perspectives
- **Infrared Cameras**: Capture heat signatures and work in low-light conditions

### How Cameras Work
Cameras capture light through a lens onto a sensor array. Each sensor element (pixel) records the intensity and color of light hitting it.

## LiDAR (Light Detection and Ranging)

LiDAR systems emit laser pulses and measure the time it takes for them to return after reflecting off objects:

- **Principle**: Time-of-flight measurement
- **Uses**: Creating detailed 3D maps of environments
- **Advantages**: Accurate distance measurements, works in various lighting conditions
- **Limitations**: Can be affected by transparent or highly reflective surfaces

## IMUs (Inertial Measurement Units)

IMUs combine multiple sensors to measure motion and orientation:

- **Accelerometer**: Measures linear acceleration
- **Gyroscope**: Measures angular velocity
- **Magnetometer**: Measures magnetic field orientation (acts like a compass)

### Applications
- Robot localization and navigation
- Balancing in humanoid robots
- Motion tracking and gesture recognition

## Integration in Robotic Systems

Robots typically use multiple sensors simultaneously to build a comprehensive understanding of their environment. This approach, called sensor fusion, allows robots to operate more reliably by combining the strengths of different sensor types.