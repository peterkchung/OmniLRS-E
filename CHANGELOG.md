# Changelog

All notable changes to this fork of OmniLRS will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Milestone 1 Complete - Dust Physics Extension

#### Phase 1: Foundation (Complete)
- **Configuration**: Added `cfg/environment/lunalab_dust_physics.yaml` with comprehensive dust physics parameters
- **Dataclass**: Implemented `DustPhysicsConf` in `src/configurations/procedural_terrain_confs.py` with full validation
- **Module Structure**: Scaffolded 6-module dust physics system in `src/environments/dust_physics/`
- **Registration**: Integrated dust physics configuration with `configFactory` system

#### Phase 2: Core Physics Implementation (Complete)
- **DustManager**: Main orchestrator managing particle lifecycle, emission, dynamics, visualization, and ROS2 publishing
- **DustEmitter**: Wheel contact detection and particle emission based on slip velocity (>0.1 m/s) and contact force (>50 N)
- **DustDynamics**: Particle physics with lunar gravity (1.62 m/s²), ballistic trajectories, drag, and empirical cohesion effects
- **DustParticle**: Optimized data structure for efficient storage and GPU operations
- **DustVisualization**: IsaacSim PointInstancer integration for particle rendering
- **DustROSPublishers**: Complete ROS2 publishers for density maps and visibility estimates

#### Phase 3: Environment Integration (Complete)
- **LunalabController Integration**: DustManager initialized in `__init__`, setup in `load()`, updated in `deform_terrain()`
- **ROS2 Integration**: Dust publishers setup via `ROS_LunalabManager` calling `dust_manager.setup_ros(node)`
- **Wheel Data Extraction**: Leverages existing wheel contact data (positions, velocities, forces) from environment controller
- **Terrain Bounds Integration**: Dust density maps respect terrain boundaries for accurate localization

#### Phase 4: Sensor Effects & Testing (Complete)
- **DustSensorEffects Module**: Comprehensive sensor degradation implementation:
  - LiDAR scatter/noise: Point dropout (up to 15%), range noise (2cm), intensity reduction (30%), false positives
  - Camera haze/degradation: Atmospheric haze, contrast reduction (40%), lunar regolith color shift
  - IMU vibration: High-frequency noise from dust impacts, velocity-scaled amplitude
- **Performance Profiling**: Built-in timing statistics for emission, dynamics, culling, visualization, and ROS publishing
- **Test Suite**: Comprehensive validation (`tests/test_dust_physics.py`) with 12 tests:
  - Particle creation, emitter functionality, dynamics (gravity, drag, cohesion)
  - Sensor effects registration and application for all sensor types
  - Integration tests for particle lifecycle and performance scaling

#### ROS2 Topics Added
- `/dust/density_map` - 2D grid of particle density for perception algorithm testing
- `/dust/visibility` - Visibility range estimate in meters for sensor noise injection

#### Configuration Parameters Added
- Sensor effect controls: `enable_sensor_effects`, `lidar_dust_noise_scale`, `camera_dust_haze_scale`, `imu_dust_vibration_scale`
- Full parameter documentation in EXTENSION.md

### Previous Changes

#### Added
- Forked from OmniLRS-E (Extended) for advanced simulation development
- Added comprehensive EXTENSION.md documenting major development areas:
  - **Dust Physics**: Based on University of Wisconsin-Madison research (Batagoda 2025, Negrut et al.) using Project Chrono
  - **ROS2 Perception Testing**: Testing platform for [juppiter](https://github.com/peterkchung/juppiter) - ROS2 perception pipeline for extreme environments
  - **Regolith Deformation**: Long-term integration of terrain deformation and sinkage physics via Chrono engine
- Added CHANGELOG.md for development tracking
- Updated README with OmniLRS-E branding and extended version notice

#### Changed
- **Simulation tuning**: Adjusted starting positions of rover, lander, and rock assets to center of terrain for improved simulation layout (`cfg/environment/lunaryard_40m_workshop.yaml`)
- **Depth camera calibration**: Modified depth camera clip range to fixed 0-10m range for consistent depth scaling, updated depth camera position on Pragyaan rover (`src/robots/yamcs_TMTC.py`, `assets/USD_Assets/robots/pragyaan/nograph_pragyaan.usd`)

#### Purpose
This fork serves as the comprehensive simulation testing platform for the juppiter ROS2 perception pipeline, focusing on extreme condition environments with the Moon as our preliminary testbed.
