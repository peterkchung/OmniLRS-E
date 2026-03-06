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

from typing import Dict, List, Optional, Tuple
import numpy as np
import time

from src.configurations.procedural_terrain_confs import DustPhysicsConf
from src.environments.dust_physics.dust_emitter import DustEmitter
from src.environments.dust_physics.dust_dynamics import DustDynamics
from src.environments.dust_physics.dust_particle import DustParticle
from src.environments.dust_physics.dust_visualization import DustVisualization
from src.environments.dust_physics.dust_ros_publishers import DustROSPublishers
from src.environments.dust_physics.dust_sensor_effects import DustSensorEffects


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
        self.ros_publishers: Optional[DustROSPublishers] = None
        self.sensor_effects: Optional[DustSensorEffects] = None

        # Particle system state
        self.particles: List = []
        self.active_particle_count: int = 0
        self.particle_pool: List = []

        # Simulation state
        self.is_enabled: bool = settings.enable
        self.simulation_time: float = 0.0
        self.terrain_bounds: Optional[Tuple[float, float, float, float]] = None

        # Performance profiling
        self.enable_profiling: bool = False
        self._profiling_data: Dict[str, List[float]] = {
            "emission": [],
            "dynamics": [],
            "culling": [],
            "visualization": [],
            "ros_publish": [],
            "total": [],
        }
        self._profiling_window: int = 100  # Keep last 100 measurements

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
        self.ros_publishers = DustROSPublishers(self.settings)
        self.sensor_effects = DustSensorEffects(self.settings)

        # Pre-allocate particle pool
        self._initialize_particle_pool()

    def setup_ros(self, node) -> None:
        """
        Setup ROS2 publishers.

        Args:
            node: ROS2 node to create publishers on.
        """
        if self.ros_publishers and self.is_enabled:
            self.ros_publishers.setup(node)

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

        total_start = time.time() if self.enable_profiling else 0

        self.simulation_time += dt

        # Step 1: Emission
        emission_start = time.time() if self.enable_profiling else 0
        if self.emitter:
            new_particles = self.emitter.emit(
                wheel_positions, wheel_velocities, wheel_forces, dt
            )
            self._add_particles(new_particles)
        if self.enable_profiling:
            self._profile_step("emission", emission_start)

        # Step 2: Dynamics
        dynamics_start = time.time() if self.enable_profiling else 0
        if self.dynamics:
            self.dynamics.update(self.particles, dt)
        if self.enable_profiling:
            self._profile_step("dynamics", dynamics_start)

        # Step 3: Culling
        culling_start = time.time() if self.enable_profiling else 0
        self._cull_expired_particles()
        if self.enable_profiling:
            self._profile_step("culling", culling_start)

        # Step 4: Visualization
        viz_start = time.time() if self.enable_profiling else 0
        if self.visualization:
            self.visualization.update(self.particles)
        if self.enable_profiling:
            self._profile_step("visualization", viz_start)

        # Step 5: ROS2 publishing
        ros_start = time.time() if self.enable_profiling else 0
        if self.ros_publishers:
            self.ros_publishers.update(
                self.particles, self.simulation_time, self.terrain_bounds
            )
        if self.enable_profiling:
            self._profile_step("ros_publish", ros_start)
            self._profile_step("total", total_start)

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

        if self.sensor_effects:
            self.sensor_effects.reset()

    def get_dust_density(self, region: Optional[tuple] = None) -> np.ndarray:
        """
        Calculate dust density in specified region.

        Args:
            region (Optional[tuple]): (x_min, x_max, y_min, y_max, resolution)
                If None, uses entire terrain bounds.

        Returns:
            np.ndarray: 2D density map (particles per cell).
        """
        if region is None and self.terrain_bounds is None:
            return np.array([])

        bounds = region if region else self.terrain_bounds
        if self.ros_publishers and bounds is not None:
            return self.ros_publishers._calculate_density_map(self.particles, bounds)
        return np.array([])

    def get_visibility_estimate(self, position: np.ndarray) -> float:
        """
        Estimate visibility at given position due to dust.

        Args:
            position (np.ndarray): [3] position in world coordinates.

        Returns:
            float: Visibility range in meters (decreases with dust density).
        """
        if self.ros_publishers:
            return self.ros_publishers._calculate_visibility(self.particles)
        return 1000.0  # Default clear visibility

    def _profile_step(self, step_name: str, start_time: float) -> None:
        """
        Record timing for a simulation step.

        Args:
            step_name (str): Name of the step being profiled.
            start_time (float): Start time from time.time().
        """
        if not self.enable_profiling:
            return

        elapsed = (time.time() - start_time) * 1000.0  # Convert to ms

        if step_name in self._profiling_data:
            self._profiling_data[step_name].append(elapsed)
            # Keep only last N measurements
            if len(self._profiling_data[step_name]) > self._profiling_window:
                self._profiling_data[step_name].pop(0)

    def get_performance_stats(self) -> Dict[str, Dict[str, float]]:
        """
        Get performance statistics for dust physics simulation.

        Returns:
            Dict[str, Dict[str, float]]: Statistics for each step:
                {
                    "emission": {"mean_ms": X, "std_ms": Y, "max_ms": Z},
                    "dynamics": {"mean_ms": X, "std_ms": Y, "max_ms": Z},
                    ...
                }
        """
        stats = {}

        for step_name, timings in self._profiling_data.items():
            if len(timings) > 0:
                arr = np.array(timings)
                stats[step_name] = {
                    "mean_ms": float(np.mean(arr)),
                    "std_ms": float(np.std(arr)),
                    "min_ms": float(np.min(arr)),
                    "max_ms": float(np.max(arr)),
                    "count": len(timings),
                }
            else:
                stats[step_name] = {
                    "mean_ms": 0.0,
                    "std_ms": 0.0,
                    "min_ms": 0.0,
                    "max_ms": 0.0,
                    "count": 0,
                }

        return stats

    def reset_performance_stats(self) -> None:
        """Reset all performance profiling data."""
        for key in self._profiling_data:
            self._profiling_data[key] = []

    def enable_performance_profiling(self, enabled: bool = True) -> None:
        """
        Enable or disable performance profiling.

        Args:
            enabled (bool): True to enable profiling, False to disable.
        """
        self.enable_profiling = enabled
        if not enabled:
            self.reset_performance_stats()
