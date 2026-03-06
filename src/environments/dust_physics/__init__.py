__author__ = "OmniLRS-E Extension"
__copyright__ = "Copyright 2025, OmniLRS-E Extension"
__license__ = "BSD 3-Clause"
__version__ = "1.0.0"
__maintainer__ = "OmniLRS-E Extension"
__email__ = ""
__status__ = "development"

"""
Dust Physics Module

Custom particle-based lunar regolith dust simulation for OmniLRS-E extension.
Implements wheel-terrain contact dust emitters with ballistic physics,
empirical cohesion effects, and ROS2 sensor integration.

This module extends OmniLRS to provide:
- Particle-based dust emitters at wheel contact points
- Medium-low fidelity dust dynamics (ballistic + drag + cohesion)
- GPU-accelerated particle system for headless operation
- ROS2 topics for dust density and visibility estimates
- Integration with LiDAR, camera, and IMU sensors

Architecture:
    dust_manager.py        - Main orchestrator, manages particle lifecycle
    dust_emitter.py          - Wheel contact detection and emission
    dust_dynamics.py         - Particle physics (ballistic, drag, cohesion)
    dust_particle.py         - Particle data structure
    dust_visualization.py    - IsaacSim rendering integration
"""

from src.environments.dust_physics.dust_manager import DustManager
from src.environments.dust_physics.dust_emitter import DustEmitter
from src.environments.dust_physics.dust_dynamics import DustDynamics
from src.environments.dust_physics.dust_particle import DustParticle
from src.environments.dust_physics.dust_visualization import DustVisualization

__all__ = [
    "DustManager",
    "DustEmitter",
    "DustDynamics",
    "DustParticle",
    "DustVisualization",
]
