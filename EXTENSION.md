# OmniLRS Extension

## Overview
This fork extends the OmniLRS lunar simulation environment to serve as a comprehensive testing platform for our ROS2 perception pipeline and package tool. We are building tooling for extreme condition environments, using the Moon as our preliminary testing ground before adapting to other planetary bodies and harsh Earth environments.

## Major Development Areas

### 1. Dust Physics Simulation
Implementation of lunar regolith dust dynamics based on the University of Wisconsin-Madison research (Batagoda 2025, Negrut et al.) and discrete element modeling approaches.

**Status: Phase 1 Complete (Foundation)**

**Key Features:**
- Custom particle-based dust emitters at wheel-terrain contact points
- Medium-fidelity dust dynamics (ballistic trajectories + drag + empirical cohesion)
- GPU-accelerated particle system for headless operation
- Accurate lunar lighting interaction with dust clouds
- Integration with rover mobility simulations
- ROS2 topics for dust density maps and visibility estimates
- LiDAR scatter, camera occlusion, and IMU vibration effects

**Implementation Details:**

The dust physics system is implemented as a custom extension within OmniLRS (not using Project Chrono), optimized for perception pipeline testing:

**Architecture:**
```
src/environments/dust_physics/
├── __init__.py              - Module exports
├── dust_manager.py          - Main orchestrator (~226 lines)
├── dust_emitter.py          - Wheel contact detection & emission (~172 lines)
├── dust_dynamics.py         - Particle physics (ballistic + drag + cohesion) (~229 lines)
├── dust_particle.py         - Particle data structure (~180 lines)
├── dust_visualization.py    - IsaacSim rendering interface (~158 lines)
├── dust_ros_publishers.py   - ROS2 topic publishing (~243 lines)
└── dust_sensor_effects.py   - Sensor degradation effects (~270 lines)
```

**Physics Model:**
- **Emission**: Triggered by wheel slip velocity (>0.1 m/s) and contact force (>50 N)
- **Ballistic motion**: Lunar gravity (1.62 m/s²), semi-implicit Euler integration
- **Drag**: Configurable drag coefficient (lunar vacuum: negligible)
- **Cohesion**: Empirical electrostatic clustering based on proximity
- **Particle properties**: 1mm to 10cm size range, 1500 kg/m³ density

**Configuration:** (`cfg/environment/lunalab_dust_physics.yaml`)
```yaml
terrain_manager:
  moon_yard:
    dust_physics:
      enable: True
      emission_rate: 1000              # particles/sec/wheel
      particle_lifetime: 2.0           # seconds
      particle_size_min: 0.001         # 1mm (fine dust)
      particle_size_max: 0.1           # 10cm (coarse)
      emission_velocity_threshold: 0.1 # m/s
      emission_force_threshold: 50.0   # N
      drag_coefficient: 0.47
      cohesion_strength: 0.01
      max_particles: 50000
      publish_dust_density: True
      dust_density_topic: "/dust/density_map"
```

**ROS2 Integration:**
- `/dust/density_map` - 2D grid of particle density (for perception algorithm testing)
- `/dust/visibility` - Visibility range estimate in meters (for sensor noise injection)

**Development Phases:**
- **Phase 1** ✅: Foundation (config, dataclass, module structure) - *COMPLETE*
- **Phase 2** ✅: Core physics implementation (emitters, dynamics, visualization) - *COMPLETE*
- **Phase 3** ✅: Environment integration (LunalabController hooks) - *COMPLETE*
- **Phase 4** ✅: ROS2 sensor integration (LiDAR scatter, camera noise, IMU vibration) - *COMPLETE*

**Sensor Effects Implementation:**

The `dust_sensor_effects.py` module provides perception degradation for ROS2 sensor pipelines:

**LiDAR Effects** (`apply_lidar_effects`):
- Point dropout in high dust regions (up to 15%)
- Range noise proportional to dust density (up to 2cm)
- Intensity reduction due to scattering (up to 30%)
- False positive points from dust reflections

**Camera Effects** (`apply_camera_effects`):
- Atmospheric haze/fog based on dust density in view frustum
- Contrast reduction (up to 40%)
- Color shift toward lunar regolith tan/grey
- Temporal accumulation for smooth transitions

**IMU Effects** (`apply_imu_effects`):
- High-frequency vibration noise from particle impacts
- Amplitude scales with robot velocity and dust density
- Temporal correlation for realistic vibration patterns
- Separate scaling for linear acceleration and angular velocity

**Configuration:**
```yaml
terrain_manager:
  moon_yard:
    dust_physics:
      # ... core physics params ...
      
      # Sensor degradation effects
      enable_sensor_effects: True
      lidar_dust_noise_scale: 1.0      # Scale factor (0.0 to disable)
      camera_dust_haze_scale: 1.0      # Scale factor (0.0 to disable)
      imu_dust_vibration_scale: 1.0    # Scale factor (0.0 to disable)
```

**API Usage:**
```python
# Register sensors for dust effects
dust_manager.sensor_effects.register_sensor(
    sensor_id="lidar_main",
    position=[1.0, 0.0, 0.5],
    orientation=[0.0, 0.0, 0.0, 1.0],  # Quaternion [x, y, z, w]
    sensor_type="lidar"
)

# Apply effects to sensor data
modified_points, modified_intensities = \
    dust_manager.sensor_effects.apply_lidar_effects(
        "lidar_main", points, intensities, particles
    )

modified_image = dust_manager.sensor_effects.apply_camera_effects(
    "camera_main", image, particles, dt
)

modified_accel, modified_gyro = \
    dust_manager.sensor_effects.apply_imu_effects(
        "imu_main", accel, gyro, particles, robot_velocity
    )
```
- Batagoda, N.M. (2025). "A physics-based simulation environment for lunar rover operations." MS Thesis, UW-Madison.
- Wang et al. (2022). "Investigating Particle-Particle Electrostatic Effects on Charged Lunar Dust Transport via Discrete Element Modeling." Advances in Space Research.
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

---

## Extension File Tree

Complete listing of OmniLRS-E extension additions:

### Configuration
```
cfg/environment/
└── lunalab_dust_physics.yaml    - Dust physics environment config
```

### Dust Physics System
```
src/environments/dust_physics/
├── __init__.py                  - Module exports and docs
├── dust_manager.py              - Main orchestrator (~350 lines)
├── dust_emitter.py              - Wheel contact detection (~172 lines)
├── dust_dynamics.py             - Particle physics (~229 lines)
├── dust_particle.py             - Particle data structure (~180 lines)
├── dust_visualization.py        - IsaacSim rendering (~158 lines)
├── dust_ros_publishers.py       - ROS2 topic publishing (~243 lines)
└── dust_sensor_effects.py       - Sensor degradation effects (~270 lines)
```

### Configuration Classes
```
src/configurations/
├── __init__.py                  - Added DustPhysicsConf export & registration
└── procedural_terrain_confs.py  - Added DustPhysicsConf class (~120 lines)
```

### Test Suite
```
tests/
└── test_dust_physics.py         - Comprehensive test suite (~650 lines)
```

**Lines Added:**
- YAML config: 200 lines
- Dust physics module: 1,802 lines
- Test suite: 650 lines
- Configuration classes: 190 lines
- Documentation: ~120 lines
- **Total: ~2,962 lines**
