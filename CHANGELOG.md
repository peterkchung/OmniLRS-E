# Changelog

All notable changes to this fork of OmniLRS will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Forked from OmniLRS-E (Extended) for advanced simulation development
- Added comprehensive EXTENSION.md documenting major development areas:
  - **Dust Physics**: Based on University of Wisconsin-Madison research (Batagoda 2025, Negrut et al.) using Project Chrono
  - **ROS2 Perception Testing**: Testing platform for [juppiter](https://github.com/peterkchung/juppiter) - ROS2 perception pipeline for extreme environments
  - **Regolith Deformation**: Long-term integration of terrain deformation and sinkage physics via Chrono engine
- Added CHANGELOG.md for development tracking
- Updated README with OmniLRS-E branding and extended version notice

### Changed
- **Simulation tuning**: Adjusted starting positions of rover, lander, and rock assets to center of terrain for improved simulation layout (`cfg/environment/lunaryard_40m_workshop.yaml`)
- **Depth camera calibration**: Modified depth camera clip range to fixed 0-10m range for consistent depth scaling, updated depth camera position on Pragyaan rover (`src/robots/yamcs_TMTC.py`, `assets/USD_Assets/robots/pragyaan/nograph_pragyaan.usd`)

### Purpose
This fork serves as the comprehensive simulation testing platform for the juppiter ROS2 perception pipeline, focusing on extreme condition environments with the Moon as our preliminary testbed.
