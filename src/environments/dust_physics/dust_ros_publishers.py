__author__ = "OmniLRS-E Extension"
__copyright__ = "Copyright 2025, OmniLRS-E Extension"
__license__ = "BSD 3-Clause"
__version__ = "1.0.0"
__maintainer__ = "OmniLRS-E Extension"
__status__ = "development"

"""
ROS2 Dust Publishers

Publishes dust simulation data for perception pipeline testing:
- Dust density maps (2D grid of particle concentration)
- Visibility estimates (range reduction due to dust)

Topics:
    /dust/density_map - GridMap message with particle density
    /dust/visibility - Float32 with estimated visibility range (m)
"""

from typing import Optional
import numpy as np

from src.configurations.procedural_terrain_confs import DustPhysicsConf
from src.environments.dust_physics.dust_particle import DustParticle


class DustROSPublishers:
    """
    ROS2 publishers for dust simulation data.

    Publishes dust density maps and visibility estimates for
    perception algorithm testing.

    Args:
        settings (DustPhysicsConf): Dust physics configuration with topic names.
    """

    def __init__(self, settings: DustPhysicsConf) -> None:
        """
        Initialize ROS2 publishers.

        Args:
            settings (DustPhysicsConf): Configuration with ROS2 topic settings.
        """
        self.settings = settings

        # Topic configuration
        self.density_topic = settings.dust_density_topic
        self.visibility_topic = settings.dust_visibility_topic
        self.publish_rate = settings.publish_rate

        # Publishers (initialized on first use if ROS2 available)
        self.density_publisher = None
        self.visibility_publisher = None
        self.is_initialized = False

        # Timing
        self.last_publish_time = 0.0

    def setup(self, node) -> None:
        """
        Setup publishers with ROS2 node.

        Args:
            node: ROS2 node to create publishers on.
        """
        if not self.settings.publish_dust_density:
            return

        try:
            from std_msgs.msg import Float32
            from grid_map_msgs.msg import GridMap

            self.density_publisher = node.create_publisher(
                GridMap, self.density_topic, 10
            )
            self.visibility_publisher = node.create_publisher(
                Float32, self.visibility_topic, 10
            )
            self.is_initialized = True
        except ImportError:
            # ROS2 not available, skip publishing
            pass

    def update(
        self,
        particles: list,
        simulation_time: float,
        terrain_bounds: Optional[tuple] = None,
    ) -> None:
        """
        Update and publish dust data.

        Args:
            particles (list): Active dust particles.
            simulation_time (float): Current simulation time.
            terrain_bounds (Optional[tuple]): (x_min, x_max, y_min, y_max) terrain bounds.
        """
        if not self.is_initialized or not self.settings.publish_dust_density:
            return

        # Check publish rate
        time_since_last = simulation_time - self.last_publish_time
        if time_since_last < (1.0 / self.publish_rate):
            return

        self.last_publish_time = simulation_time

        # Calculate and publish density map
        if terrain_bounds:
            density_map = self._calculate_density_map(particles, terrain_bounds)
            self._publish_density_map(density_map, terrain_bounds)

        # Calculate and publish visibility estimate
        visibility = self._calculate_visibility(particles)
        self._publish_visibility(visibility)

    def _calculate_density_map(
        self, particles: list, terrain_bounds: tuple, resolution: float = 0.5
    ) -> np.ndarray:
        """
        Calculate 2D dust density map.

        Args:
            particles (list): Active dust particles.
            terrain_bounds (tuple): (x_min, x_max, y_min, y_max).
            resolution (float): Grid cell size in meters.

        Returns:
            np.ndarray: 2D array of particle counts per cell.
        """
        x_min, x_max, y_min, y_max = terrain_bounds

        # Create grid
        x_cells = int((x_max - x_min) / resolution)
        y_cells = int((y_max - y_min) / resolution)
        density_map = np.zeros((y_cells, x_cells), dtype=np.int32)

        # Count particles in each cell
        for particle in particles:
            x, y, z = particle.position

            # Map to grid indices
            x_idx = int((x - x_min) / resolution)
            y_idx = int((y - y_min) / resolution)

            # Check bounds
            if 0 <= x_idx < x_cells and 0 <= y_idx < y_cells:
                density_map[y_idx, x_idx] += 1

        return density_map

    def _publish_density_map(
        self, density_map: np.ndarray, terrain_bounds: tuple
    ) -> None:
        """
        Publish density map as GridMap message.

        Args:
            density_map (np.ndarray): 2D particle density.
            terrain_bounds (tuple): (x_min, x_max, y_min, y_max).
        """
        if self.density_publisher is None:
            return

        try:
            from grid_map_msgs.msg import GridMap
            from std_msgs.msg import Float32MultiArray, MultiArrayDimension

            x_min, x_max, y_min, y_max = terrain_bounds

            # Create GridMap message
            msg = GridMap()
            msg.info.header.frame_id = "world"
            msg.info.resolution = 0.5  # meters per cell
            msg.info.length_x = x_max - x_min
            msg.info.length_y = y_max - y_min
            msg.info.pose.position.x = (x_min + x_max) / 2.0
            msg.info.pose.position.y = (y_min + y_max) / 2.0

            # Add density layer
            density_data = Float32MultiArray()
            density_data.data = density_map.flatten().astype(np.float32).tolist()
            density_data.layout.dim = [
                MultiArrayDimension(label="y", size=density_map.shape[0]),
                MultiArrayDimension(label="x", size=density_map.shape[1]),
            ]

            msg.layers = ["dust_density"]
            msg.data = [density_data]

            self.density_publisher.publish(msg)
        except Exception:
            # ROS2 publishing failed, skip
            pass

    def _calculate_visibility(self, particles: list) -> float:
        """
        Calculate estimated visibility range due to dust.

        Visibility decreases with particle density. Based on
        Beer-Lambert law approximation: visibility = constant / density.

        Args:
            particles (list): Active dust particles.

        Returns:
            float: Estimated visibility range in meters.
        """
        if len(particles) == 0:
            return 1000.0  # Clear visibility (1km)

        # Calculate average particle density per cubic meter
        # Assume simulation volume of 100x100x10 meters (conservative)
        simulation_volume = 100.0 * 100.0 * 10.0  # m^3
        particle_density = len(particles) / simulation_volume

        # Visibility model: higher density = lower visibility
        # Base visibility in clear conditions: 1000m
        # At 1000 particles/m^3: visibility ~50m
        base_visibility = 1000.0
        visibility = base_visibility / (1.0 + particle_density * 0.05)

        return max(visibility, 10.0)  # Minimum 10m visibility

    def _publish_visibility(self, visibility: float) -> None:
        """
        Publish visibility estimate.

        Args:
            visibility (float): Estimated visibility range in meters.
        """
        if self.visibility_publisher is None:
            return

        try:
            from std_msgs.msg import Float32

            msg = Float32()
            msg.data = visibility
            self.visibility_publisher.publish(msg)
        except Exception:
            pass
