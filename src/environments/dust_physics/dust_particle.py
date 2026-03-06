__author__ = "OmniLRS-E Extension"
__copyright__ = "Copyright 2025, OmniLRS-E Extension"
__license__ = "BSD 3-Clause"
__version__ = "1.0.0"
__maintainer__ = "OmniLRS-E Extension"
__status__ = "development"

"""
DustParticle - Data structure for individual dust particles

Represents state and properties of a single dust particle in the simulation.
Optimized for efficient storage and GPU operations.
"""

import numpy as np
from typing import Optional


class DustParticle:
    """
    Represents a single dust particle in the simulation.

    Stores position, velocity, size, and lifetime information.
    Designed for efficient array operations and GPU compatibility.

    Args:
        position (np.ndarray): [3] Initial position [x, y, z] in meters.
        velocity (np.ndarray): [3] Initial velocity [vx, vy, vz] in m/s.
        size (float): Particle diameter in meters.
        density (float): Particle density in kg/m^3.
        lifetime (float): Maximum lifetime in seconds.
        creation_time (float): Simulation time when particle was created.
    """

    def __init__(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        size: float,
        density: float,
        lifetime: float,
        creation_time: float,
    ) -> None:
        """
        Initialize particle state.

        Args:
            position (np.ndarray): [3] position in meters.
            velocity (np.ndarray): [3] velocity in m/s.
            size (float): Diameter in meters.
            density (float): Density in kg/m^3.
            lifetime (float): Maximum lifetime in seconds.
            creation_time (float): Simulation time at creation.
        """
        # Spatial properties
        self.position = np.array(position, dtype=np.float32)
        self.velocity = np.array(velocity, dtype=np.float32)

        # Physical properties
        self.size = float(size)
        self.density = float(density)
        self.mass = self._calculate_mass()

        # Cross-sectional area for drag calculations
        self.cross_sectional_area = np.pi * (self.size / 2.0) ** 2

        # Temporal properties
        self.creation_time = float(creation_time)
        self.lifetime = float(lifetime)

        # State flags
        self.is_settled = False
        self.settlement_time: Optional[float] = None

    def _calculate_mass(self) -> float:
        """
        Calculate particle mass from size and density.

        Assumes spherical particle: mass = density * (4/3) * pi * r^3

        Returns:
            float: Particle mass in kg.
        """
        radius = self.size / 2.0
        volume = (4.0 / 3.0) * np.pi * radius**3
        return self.density * volume

    def is_active(self, current_time: float, settling_velocity: float) -> bool:
        """
        Check if particle is still active in simulation.

        Particle is inactive when:
        - Age exceeds lifetime
        - Has settled (velocity < settling_velocity)
        - Position below terrain surface (handled externally)

        Args:
            current_time (float): Current simulation time.
            settling_velocity (float): Velocity threshold for settling.

        Returns:
            bool: True if particle is still active.
        """
        # Check age
        age = current_time - self.creation_time
        if age >= self.lifetime:
            return False

        # Check if settled
        speed = np.linalg.norm(self.velocity)
        if speed < settling_velocity:
            if not self.is_settled:
                self.is_settled = True
                self.settlement_time = current_time
            return False

        return True

    def update_position(self, dt: float) -> None:
        """
        Update particle position based on velocity.

        Simple Euler integration: p_new = p_old + v * dt

        Args:
            dt (float): Time step in seconds.
        """
        self.position += self.velocity * dt

    def get_age(self, current_time: float) -> float:
        """
        Get particle age.

        Args:
            current_time (float): Current simulation time.

        Returns:
            float: Age in seconds.
        """
        return current_time - self.creation_time

    def to_array(self) -> np.ndarray:
        """
        Convert particle to numpy array for GPU operations.

        Returns:
            np.ndarray: Flattened particle state [position(3), velocity(3), size(1), density(1)].
        """
        return np.concatenate(
            [self.position, self.velocity, [self.size, self.density]]
        ).astype(np.float32)

    @classmethod
    def from_array(cls, arr: np.ndarray, creation_time: float) -> "DustParticle":
        """
        Create particle from numpy array.

        Args:
            arr (np.ndarray): Flattened particle state [position(3), velocity(3), size(1), density(1)].
            creation_time (float): Creation time for new particle.

        Returns:
            DustParticle: Reconstructed particle.
        """
        position = arr[0:3]
        velocity = arr[3:6]
        size = arr[6]
        density = arr[7]

        # Default lifetime (can be overridden)
        lifetime = 2.0

        return cls(
            position=position,
            velocity=velocity,
            size=size,
            density=density,
            lifetime=lifetime,
            creation_time=creation_time,
        )
