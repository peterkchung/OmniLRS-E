# OmniLRS Extension

## Overview
This fork extends the OmniLRS lunar simulation environment to serve as a comprehensive testing platform for our ROS2 perception pipeline and package tool. We are building tooling for extreme condition environments, using the Moon as our preliminary testing ground before adapting to other planetary bodies and harsh Earth environments.

## Major Development Areas

### 1. Dust Physics Simulation
Implementation of lunar regolith dust dynamics based on the University of Wisconsin-Madison research (Batagoda 2025, Negrut et al.) using Project Chrono.

**Key Features:**
- Particle-based dust emitters at wheel-terrain contact points
- Physics-based dust dynamics using Chrono::Engine
- Accurate lunar lighting interaction with dust clouds
- Integration with rover mobility simulations

**References:**
- Batagoda, N.M. (2025). "A physics-based simulation environment for lunar rover operations." MS Thesis, UW-Madison.
- "A Physics-Based Sensor Simulation Environment for Lunar Ground Operations" (2025 IEEE Aerospace Conference)

### 2. ROS2 Perception Pipeline Testing Environment
Primary testing platform for [juppiter](https://github.com/peterkchung/juppiter) - our ROS2 perception package and tool development framework for extreme condition environments.

**Testing Capabilities:**
- Camera sensor validation (RGB, depth, thermal)
- LiDAR point cloud processing and ground segmentation
- Visual odometry and SLAM algorithm testing
- Object detection and tracking in low-visibility conditions
- Sim-to-real validation workflows
- Ground truth generation for perception algorithm benchmarking
- Multi-modal sensor fusion testing

**Application:**
Using the Moon as our preliminary testbed for developing perception tooling that can be adapted to:
- Other planetary bodies (Mars, asteroids)
- Harsh Earth environments (polar regions, deserts, underwater)
- Extreme industrial conditions (mining, disaster response)

### 3. Regolith Deformation & Sinkage Physics (Long-term)
Advanced terrain deformation and wheel sinkage modeling using Project Chrono physics engine. This represents a significant longer-term integration effort.

**Technical Goals:**
- Physics-based terramechanics using smoothed particle hydrodynamics (SPH)
- Wheel-soil interaction modeling for accurate sinkage prediction
- Reduced-gravity granular dynamics
- Integration with Chrono::Worlds for large-scale lunar environments

**Implementation Strategy:**
- Phase 1: Research and proof-of-concept Chrono integration
- Phase 2: Granular DEM validation for lunar regolith simulants
- Phase 3: Full terrain deformation system integration
- Phase 4: Performance optimization for real-time simulation

**Draws on:**
- Batagoda, N.M. (2025) - CRM terramechanics and SPH methods
- UW-Madison SBEL research on lunar rover mobility analysis

## References
- Batagoda, N.M. (2025). "A physics-based simulation environment for lunar rover operations." University of Wisconsin-Madison.
- Negrut et al. (2025). "A Physics-Based Sensor Simulation Environment for Lunar Ground Operations." IEEE Aerospace Conference.
- University of Wisconsin-Madison SBEL: https://sbel.wisc.edu
- Project Chrono: https://projectchrono.org

## Acknowledgments
This extension builds upon research from the Simulation-Based Engineering Laboratory (SBEL) at the University of Wisconsin-Madison, supported by NASA STTR Phase II grant 80NSSC24CA030 and NSF project OAC220979.
