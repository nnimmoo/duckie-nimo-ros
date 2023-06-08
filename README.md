# Project Duckiebot Avoidance

This project aims to enable a DuckieBot to detect other DuckieBots or ducks in its driving lane and navigate around them to ensure safe and uninterrupted driving.

## How to use it

### 1. Introduction

The DuckieBot Lane Navigation project addresses the challenge of detecting obstacles, specifically DuckieBots or ducks, in the driving lane of a DuckieBot. It implements a solution that enables the DuckieBot to autonomously navigate around these obstacles, ensuring a safe and uninterrupted driving experience.

The project relies on computer vision techniques for object detection and a control algorithm for navigation. By integrating these components, the DuckieBot can identify potential obstacles and take appropriate actions to avoid collisions, thereby enhancing the overall safety and efficiency of its navigation.


### 2. Requirements

To run this project, you need the following:

- A DuckieBot equipped with a camera
- A compatible operating system (e.g., Duckietown, ROS, or any system supporting the Duckietown stack)
- The necessary software dependencies (docker, duckietown)

### 3. Instalation and Usage
To install and set up the project, follow these steps:
Clone the project repository:
```
git clone https://github.com/nnimmoo/duckie-nimo-ros
```
after that, navigate to the project directory and run the following codes:
```
dts devel build -f
dts devel run
```
To run the project on the duckiebot, use following ones:
```
dts devel build -f -H ![MY_ROBOT].local
dts devel run -H ![MY_ROBOT].local
```
(use robots name instead of **![MY_ROBOT]**)
