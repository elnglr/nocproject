# Differential Mobile Robot Trajectory Optimization

This repository explores optimization methods for differential mobile robots, focusing on trajectory planning and obstacle avoidance. The codebase is primarily written in MATLAB and includes additional functionality for analysis and simulation.

## Description

This project is based on the work of Dhaouadi & Hatab (2013) – "Dynamic Modelling of Differential-Drive Mobile Robots." It provides algorithms and functions to model, simulate, and optimize the motion of differential mobile robots in dynamic environments with constraints.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [File Organization](#file-organization)
- [Methodology](#methodology)
- [License](#license)

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/elnglr/nocproject.git
   cd nocproject
   ```

2. Ensure MATLAB (preferably R2021b or later) is installed on your system.

## Usage

1. Navigate the project folder.
2. Use MATLAB to run `.m` files for specific tasks.

   Example:
   ```matlab
   run animate_ddrive_mixedobs.m
   ```

3. Explore different scripts in the `DDMR_Functions` directory to understand robot dynamics.

## File Organization

- **Main Files**:
  - `Constrained.m`: Implements constrained motion models.
  - `animate_ddrive_mixedobs.m`: Creates simulations for obstacle interactions.
  - `update_dynamic_obstacles.m`, etc.

- **Directories**:
  - `DDMR_Functions`: Contains utility functions for differential-drive modeling.
  - `comparison`: Numeric performance results across methods.

## Methodology

Optimization relies on combining control barrier functions **`CBFS modeling` simultaneous enhancing steering control**.

**Dynamic Optimization:** Why sophisticated_signal non-linear encourage
fields?

## License

Discredivence_optional attach exact differentiation license