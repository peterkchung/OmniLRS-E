__author__ = "OmniLRS-E Extension"
__copyright__ = "Copyright 2025, OmniLRS-E Extension"
__license__ = "BSD 3-Clause"
__version__ = "1.0.0"
__maintainer__ = "OmniLRS-E Extension"
__status__ = "development"

"""
DustDynamics - Particle physics simulation

Implements ballistic trajectories with drag, lunar gravity,
and empirical cohesion effects for lunar regolith dust.

Physics model:
- Ballistic motion: v_new = v_old + g * dt
- Drag force: F_drag = -0.5 * rho * Cd * A * |v| * v
- Cohesion: F_cohesion = empirical clustering effect
"""

from typing import List
import numpy as np

from src.configurations.procedural_terrain_confs import DustPhysicsConf
from src.environments.dust_physics.dust_particle import DustParticle


class DustDynamics:
    """
    Particle dynamics simulator for lunar dust.

    Updates particle velocities and positions based on:
    - Lunar gravity (1/6 Earth)
    - Aerodynamic drag
    - Empirical cohesion effects

    Args:
        settings (DustPhysicsConf): Dust physics configuration.
    """

    def __init__(self, settings: DustPhysicsConf) -> None:
        """
        Initialize dynamics with configuration.

        Args:
            settings (DustPhysicsConf): Dust physics configuration.
        """
        self.settings = settings

        # Physics parameters
        self.gravity = np.array(settings.gravity)
        self.drag_coefficient = settings.drag_coefficient
        self.cohesion_strength = settings.cohesion_strength

        # Atmospheric density (lunar vacuum approximation)
        # Not actually used on Moon but kept for extensibility to Mars
        self.atmospheric_density = 0.0  # kg/m^3 (lunar vacuum)

    def update(self, particles: List[DustParticle], dt: float) -> None:
        """
        Update particle positions and velocities.

        Applies forces and integrates motion for all active particles.
        Uses semi-implicit Euler integration for stability.

        Args:
            particles (List[DustParticle]): Active particles to update.
            dt (float): Time step in seconds.
        """
        for particle in particles:
            if particle.is_settled:
                continue

            # Calculate forces
            gravity_force = self._calculate_gravity(particle)
            drag_force = self._calculate_drag(particle)
            cohesion_force = self._calculate_cohesion(particle, particles)

            # Net force
            net_force = gravity_force + drag_force + cohesion_force

            # Update velocity (F = ma -> a = F/m)
            acceleration = net_force / particle.mass
            particle.velocity += acceleration * dt

            # Update position
            particle.update_position(dt)

            # Check for terrain collision (ground contact)
            if particle.position[2] < 0:  # Below terrain
                particle.position[2] = 0
                particle.velocity[2] = 0
                particle.is_settled = True

    def _calculate_gravity(self, particle: DustParticle) -> np.ndarray:
        """
        Calculate gravitational force on particle.

        Args:
            particle (DustParticle): Particle to calculate force for.

        Returns:
            np.ndarray: [3] Gravity force vector [Fx, Fy, Fz].
        """
        return particle.mass * self.gravity

    def _calculate_drag(self, particle: DustParticle) -> np.ndarray:
        """
        Calculate aerodynamic drag force.

        Drag equation: F = -0.5 * rho * Cd * A * |v|^2 * (v/|v|)
        Simplified: F = -0.5 * rho * Cd * A * |v| * v

        On the Moon (vacuum), drag is negligible but we include it
        for extensibility to Mars simulations.

        Args:
            particle (DustParticle): Particle to calculate force for.

        Returns:
            np.ndarray: [3] Drag force vector [Fx, Fy, Fz].
        """
        if self.atmospheric_density == 0:
            return np.zeros(3)

        velocity_magnitude = np.linalg.norm(particle.velocity)
        if velocity_magnitude < 1e-6:
            return np.zeros(3)

        drag_magnitude = (
            0.5
            * self.atmospheric_density
            * self.drag_coefficient
            * particle.cross_sectional_area
            * velocity_magnitude**2
        )

        # Drag opposes velocity
        drag_direction = -particle.velocity / velocity_magnitude

        return drag_magnitude * drag_direction

    def _calculate_cohesion(
        self, particle: DustParticle, particles: List[DustParticle]
    ) -> np.ndarray:
        """
        Calculate empirical cohesion force.

        Lunar dust exhibits electrostatic cohesion effects causing
        particles to cluster. We use an empirical model based on
        proximity to other particles.

        Args:
            particle (DustParticle): Target particle.
            particles (List[DustParticle]): All active particles.

        Returns:
            np.ndarray: [3] Cohesion force vector.
        """
        if self.cohesion_strength == 0:
            return np.zeros(3)

        cohesion_force = np.zeros(3)
        cohesion_radius = particle.size * 5.0  # Interaction radius

        for other in particles:
            if other is particle:
                continue

            displacement = other.position - particle.position
            distance = np.linalg.norm(displacement)

            if distance < cohesion_radius and distance > 1e-6:
                # Attractive force proportional to cohesion strength
                # and inverse to distance
                force_magnitude = (
                    self.cohesion_strength * particle.mass * other.mass / (distance**2)
                )

                direction = displacement / distance
                cohesion_force += force_magnitude * direction

        return cohesion_force

    def batch_update(
        self,
        positions: np.ndarray,
        velocities: np.ndarray,
        masses: np.ndarray,
        sizes: np.ndarray,
        dt: float,
    ) -> tuple:
        """
        Batch update for GPU acceleration.

        Vectorized update for efficient GPU computation.
        Returns updated positions and velocities.

        Args:
            positions (np.ndarray): [N, 3] particle positions.
            velocities (np.ndarray): [N, 3] particle velocities.
            masses (np.ndarray): [N] particle masses.
            sizes (np.ndarray): [N] particle sizes.
            dt (float): Time step.

        Returns:
            tuple: (new_positions, new_velocities) as np.ndarray.
        """
        # Calculate gravitational acceleration (same for all)
        gravity_accel = self.gravity  # [3]

        # Calculate drag (simplified for batch)
        # For now, skip drag in batch mode (lunar vacuum)
        drag_accel = np.zeros_like(velocities)

        # Total acceleration
        acceleration = gravity_accel + drag_accel

        # Update velocities: v_new = v_old + a * dt
        new_velocities = velocities + acceleration * dt

        # Update positions: p_new = p_old + v_new * dt
        new_positions = positions + new_velocities * dt

        # Ground collision check
        ground_mask = new_positions[:, 2] < 0
        new_positions[ground_mask, 2] = 0
        new_velocities[ground_mask, 2] = 0

        return new_positions, new_velocities
