__author__ = "OmniLRS-E Extension"
__copyright__ = "Copyright 2025, OmniLRS-E Extension"
__license__ = "BSD 3-Clause"
__version__ = "1.0.0"
__maintainer__ = "OmniLRS-E Extension"
__status__ = "development"

"""
DustEmitter - Wheel contact detection and particle emission

Detects wheel-terrain contacts and emits dust particles based on
slip velocity and contact force thresholds.
"""

from typing import List
import numpy as np

from src.configurations.procedural_terrain_confs import DustPhysicsConf
from src.environments.dust_physics.dust_particle import DustParticle


class DustEmitter:
    """
    Dust particle emitter for wheel-terrain contacts.

    Monitors wheel slip velocities and contact forces to determine
    when to emit dust particles. Emission follows UW-Madison research
    patterns with configurable thresholds.

    Args:
        settings (DustPhysicsConf): Dust physics configuration.
    """

    def __init__(self, settings: DustPhysicsConf) -> None:
        """
        Initialize emitter with configuration.

        Args:
            settings (DustPhysicsConf): Dust physics configuration.
        """
        self.settings = settings

        # Emission parameters
        self.emission_rate = settings.emission_rate
        self.velocity_threshold = settings.emission_velocity_threshold
        self.force_threshold = settings.emission_force_threshold

        # Particle properties
        self.size_min = settings.particle_size_min
        self.size_max = settings.particle_size_max
        self.density = settings.particle_density
        self.lifetime = settings.particle_lifetime
        self.gravity = np.array(settings.gravity)

    def emit(
        self,
        wheel_positions: np.ndarray,
        wheel_velocities: np.ndarray,
        wheel_forces: np.ndarray,
        dt: float,
    ) -> List[DustParticle]:
        """
        Emit dust particles based on wheel contact conditions.

        Checks each wheel for emission conditions:
        - Slip velocity > threshold
        - Contact force > threshold

        Emits particles at contact point with initial velocity
        based on wheel slip direction.

        Args:
            wheel_positions (np.ndarray): [N, 3] wheel contact positions.
            wheel_velocities (np.ndarray): [N, 3] wheel slip velocities (m/s).
            wheel_forces (np.ndarray): [N, 3] wheel contact forces (N).
            dt (float): Time step in seconds.

        Returns:
            List[DustParticle]: Newly emitted particles.
        """
        new_particles = []

        # Calculate number of particles to emit this frame
        particles_per_frame = int(self.emission_rate * dt)

        for i in range(len(wheel_positions)):
            # Check emission conditions
            slip_speed = np.linalg.norm(wheel_velocities[i])
            contact_force = np.linalg.norm(wheel_forces[i])

            if slip_speed < self.velocity_threshold:
                continue
            if contact_force < self.force_threshold:
                continue

            # Emit particles for this wheel
            num_particles = self._calculate_emission_count(
                slip_speed, contact_force, particles_per_frame
            )

            for _ in range(num_particles):
                particle = self._create_particle(
                    wheel_positions[i], wheel_velocities[i], wheel_forces[i]
                )
                new_particles.append(particle)

        return new_particles

    def _calculate_emission_count(
        self, slip_speed: float, contact_force: float, base_count: int
    ) -> int:
        """
        Calculate number of particles to emit based on conditions.

        Args:
            slip_speed (float): Wheel slip velocity (m/s).
            contact_force (float): Wheel contact force (N).
            base_count (int): Base particles per frame.

        Returns:
            int: Number of particles to emit.
        """
        # Scale emission with slip velocity and force
        velocity_scale = min(slip_speed / self.velocity_threshold, 3.0)
        force_scale = min(contact_force / self.force_threshold, 2.0)

        return int(base_count * velocity_scale * force_scale)

    def _create_particle(
        self, position: np.ndarray, velocity: np.ndarray, force: np.ndarray
    ) -> DustParticle:
        """
        Create a single dust particle with randomized properties.

        Args:
            position (np.ndarray): [3] emission position.
            velocity (np.ndarray): [3] wheel slip velocity.
            force (np.ndarray): [3] contact force direction.

        Returns:
            DustParticle: New particle with randomized size and initial velocity.
        """
        # Random size within range (log-uniform for realistic dust distribution)
        log_min = np.log(self.size_min)
        log_max = np.log(self.size_max)
        log_size = np.random.uniform(log_min, log_max)
        size = np.exp(log_size)

        # Initial position with slight randomization around contact point
        position_jitter = np.random.normal(0, size * 0.5, 3)
        initial_position = position + position_jitter

        # Initial velocity based on wheel slip with upward component
        # Dust is kicked up primarily in slip direction with some vertical lift
        slip_direction = velocity / (np.linalg.norm(velocity) + 1e-6)
        upward_component = np.array([0, 0, 1.0]) * np.random.uniform(0.5, 2.0)

        initial_velocity = slip_direction * np.linalg.norm(
            velocity
        ) * 0.7 + upward_component * np.random.uniform(0.3, 1.0)

        # Randomized lifetime within range
        lifetime = np.random.uniform(self.lifetime * 0.5, self.lifetime * 1.5)

        return DustParticle(
            position=initial_position,
            velocity=initial_velocity,
            size=size,
            density=self.density,
            lifetime=lifetime,
            creation_time=0.0,  # Set by manager
        )
