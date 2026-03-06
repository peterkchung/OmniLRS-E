__author__ = "OmniLRS-E Extension"
__copyright__ = "Copyright 2025, OmniLRS-E Extension"
__license__ = "BSD 3-Clause"
__version__ = "1.0.0"
__maintainer__ = "OmniLRS-E Extension"
__status__ = "development"

"""
DustManager - Main orchestrator for dust physics simulation

Manages particle lifecycle, coordinates emitters and dynamics,
and interfaces with the environment controller (Lunalab/Lunaryard).

Pattern follows DeformationEngine from terrain_management module.
"""

from typing import Dict, List, Optional
import numpy as np

from src.configurations.procedural_terrain_confs import DustPhysicsConf
from src.environments.dust_physics.dust_emitter import DustEmitter
from src.environments.dust_physics.dust_dynamics import DustDynamics
from src.environments.dust_physics.dust_particle import DustParticle
from src.environments.dust_physics.dust_visualization import DustVisualization


class DustManager:
    """
    Main dust physics manager class.

    Coordinates particle emission, dynamics updates, and visualization
    for lunar regolith dust simulation.

    Args:
        settings (DustPhysicsConf): Configuration for dust physics parameters.
        world: IsaacSim world instance for rendering integration.
    """

    def __init__(self, settings: DustPhysicsConf, world=None) -> None:
        """
        Initialize dust manager with configuration.

        Args:
            settings (DustPhysicsConf): Dust physics configuration.
            world: IsaacSim world instance (optional, for rendering).
        """
        self.settings = settings
        self.world = world

        # Component modules (initialized in setup)
        self.emitter: Optional[DustEmitter] = None
        self.dynamics: Optional[DustDynamics] = None
        self.visualization: Optional[DustVisualization] = None

        # Particle system state
        self.particles: List[DustParticle] = []
        self.active_particle_count: int = 0
        self.particle_pool: np.ndarray = None  # Pre-allocated GPU/CPU array

        # Simulation state
        self.is_enabled: bool = settings.enable
        self.simulation_time: float = 0.0

    def setup(self) -> None:
        """
        Initialize particle system and submodules.

        Called after world is initialized. Sets up:
        - Particle pool (pre-allocated for performance)
        - Emitter, dynamics, and visualization modules
        - ROS2 publishers (if ROS2 is enabled)
        """
        if not self.is_enabled:
            return

        # Initialize submodules
        self.emitter = DustEmitter(self.settings)
        self.dynamics = DustDynamics(self.settings)
        self.visualization = DustVisualization(self.settings, self.world)

        # Pre-allocate particle pool
        self._initialize_particle_pool()

    def _initialize_particle_pool(self) -> None:
        """
        Pre-allocate particle pool for GPU/CPU operations.

        Creates array of max_particles DustParticle objects.
        Enables efficient particle recycling without allocation overhead.
        """
        # TODO: Implement GPU-backed particle pool
        # For now, use Python list (to be optimized for GPU)
        self.particle_pool = [None] * self.settings.max_particles
        self.active_particle_count = 0

    def update(
        self,
        wheel_positions: np.ndarray,
        wheel_velocities: np.ndarray,
        wheel_forces: np.ndarray,
        dt: float,
    ) -> None:
        """
        Main update loop for dust physics.

        Called at each physics step. Performs:
        1. Emission: Check wheel contacts and emit new particles
        2. Dynamics: Update particle positions and velocities
        3. Culling: Remove expired/settled particles
        4. Visualization: Update rendering
        5. ROS2: Publish density maps and visibility

        Args:
            wheel_positions (np.ndarray): [N, 3] wheel contact positions.
            wheel_velocities (np.ndarray): [N, 3] wheel slip velocities.
            wheel_forces (np.ndarray): [N, 3] wheel contact forces.
            dt (float): Time step in seconds.
        """
        if not self.is_enabled:
            return

        self.simulation_time += dt

        # Step 1: Emission
        new_particles = self.emitter.emit(
            wheel_positions, wheel_velocities, wheel_forces, dt
        )
        self._add_particles(new_particles)

        # Step 2: Dynamics
        self.dynamics.update(self.particles, dt)

        # Step 3: Culling
        self._cull_expired_particles()

        # Step 4: Visualization
        if self.visualization:
            self.visualization.update(self.particles)

        # Step 5: ROS2 publishing
        # TODO: Implement ROS2 publishers for density/visibility

    def _add_particles(self, new_particles: List[DustParticle]) -> None:
        """
        Add newly emitted particles to active pool.

        Args:
            new_particles (List[DustParticle]): Particles to add.
        """
        # Add particles up to max limit
        available_slots = self.settings.max_particles - self.active_particle_count
        particles_to_add = min(len(new_particles), available_slots)

        for i in range(particles_to_add):
            self.particle_pool[self.active_particle_count + i] = new_particles[i]

        self.active_particle_count += particles_to_add
        self.particles = self.particle_pool[: self.active_particle_count]

    def _cull_expired_particles(self) -> None:
        """
        Remove expired or settled particles from active pool.

        Particles are removed when:
        - Age exceeds particle_lifetime
        - Velocity below settling_threshold
        - Position below terrain surface
        """
        if self.active_particle_count == 0:
            return

        # Filter active particles
        active = []
        for i in range(self.active_particle_count):
            particle = self.particle_pool[i]
            if particle.is_active(
                self.simulation_time, self.settings.settling_velocity
            ):
                active.append(particle)

        # Update pool
        self.active_particle_count = len(active)
        for i, particle in enumerate(active):
            self.particle_pool[i] = particle
        self.particles = self.particle_pool[: self.active_particle_count]

    def reset(self) -> None:
        """
        Reset dust simulation state.

        Clears all particles and resets simulation time.
        Called during environment reset.
        """
        self.particles = []
        self.active_particle_count = 0
        self.simulation_time = 0.0

        if self.visualization:
            self.visualization.clear()

    def get_dust_density(self, region: Optional[tuple] = None) -> np.ndarray:
        """
        Calculate dust density in specified region.

        Args:
            region (Optional[tuple]): (x_min, x_max, y_min, y_max, resolution)
                If None, uses entire terrain bounds.

        Returns:
            np.ndarray: 2D density map (particles per cell).
        """
        # TODO: Implement density grid calculation
        pass

    def get_visibility_estimate(self, position: np.ndarray) -> float:
        """
        Estimate visibility at given position due to dust.

        Args:
            position (np.ndarray): [3] position in world coordinates.

        Returns:
            float: Visibility range in meters (decreases with dust density).
        """
        # TODO: Implement visibility calculation based on particle density
        pass
