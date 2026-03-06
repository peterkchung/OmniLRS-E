__author__ = "OmniLRS-E Extension"
__copyright__ = "Copyright 2025, OmniLRS-E Extension"
__license__ = "BSD 3-Clause"
__version__ = "1.0.0"
__maintainer__ = "OmniLRS-E Extension"
__status__ = "development"

"""
DustSensorEffects - Sensor degradation effects from lunar dust

Implements perception degradation for ROS2 sensor pipelines:
- LiDAR point cloud scatter and noise injection
- Camera image quality degradation (haze, blur, occlusion)
- IMU vibration effects from dust impacts

Physics-based effects scale with dust density and particle proximity
to sensors for realistic perception testing.
"""

from typing import Dict, List, Optional, Tuple, Any
import numpy as np
from scipy.spatial.transform import Rotation as R

from src.configurations.procedural_terrain_confs import DustPhysicsConf
from src.environments.dust_physics.dust_particle import DustParticle


class DustSensorEffects:
    """
    Sensor degradation simulator for dust-affected perception.

    Applies physics-based noise and artifacts to sensor data based on
    dust particle density and proximity. Designed for testing perception
    pipelines under degraded visibility conditions.

    Args:
        settings (DustPhysicsConf): Dust physics configuration.
    """

    def __init__(self, settings: DustPhysicsConf) -> None:
        """
        Initialize sensor effects with configuration.

        Args:
            settings (DustPhysicsConf): Dust physics configuration.
        """
        self.settings = settings

        # Effect intensity scaling
        self.lidar_noise_scale = settings.lidar_dust_noise_scale
        self.camera_haze_scale = settings.camera_dust_haze_scale
        self.imu_vibration_scale = settings.imu_dust_vibration_scale

        # Sensor state tracking
        self.sensor_positions: Dict[str, np.ndarray] = {}
        self.sensor_orientations: Dict[str, np.ndarray] = {}
        self.sensor_active: Dict[str, bool] = {}

        # Effect accumulation for temporal coherence
        self._camera_haze_accumulation: Dict[str, float] = {}
        self._imu_vibration_history: Dict[str, List[np.ndarray]] = {}
        self._history_max_len = 10

    def register_sensor(
        self,
        sensor_id: str,
        position: np.ndarray,
        orientation: np.ndarray,
        sensor_type: str,
    ) -> None:
        """
        Register a sensor for dust effect application.

        Args:
            sensor_id (str): Unique sensor identifier.
            position (np.ndarray): [3] Sensor position in world coordinates.
            orientation (np.ndarray): [4] Sensor orientation quaternion [x, y, z, w].
            sensor_type (str): Type of sensor ('lidar', 'camera', 'imu').
        """
        self.sensor_positions[sensor_id] = np.array(position, dtype=np.float32)
        self.sensor_orientations[sensor_id] = np.array(orientation, dtype=np.float32)
        self.sensor_active[sensor_id] = True

        # Initialize effect accumulation
        self._camera_haze_accumulation[sensor_id] = 0.0
        self._imu_vibration_history[sensor_id] = []

    def unregister_sensor(self, sensor_id: str) -> None:
        """
        Remove a sensor from dust effect tracking.

        Args:
            sensor_id (str): Sensor identifier to remove.
        """
        if sensor_id in self.sensor_positions:
            del self.sensor_positions[sensor_id]
            del self.sensor_orientations[sensor_id]
            del self.sensor_active[sensor_id]
            if sensor_id in self._camera_haze_accumulation:
                del self._camera_haze_accumulation[sensor_id]
            if sensor_id in self._imu_vibration_history:
                del self._imu_vibration_history[sensor_id]

    def update_sensor_pose(
        self, sensor_id: str, position: np.ndarray, orientation: np.ndarray
    ) -> None:
        """
        Update sensor pose for dynamic dust effect calculation.

        Args:
            sensor_id (str): Sensor identifier.
            position (np.ndarray): [3] Updated position.
            orientation (np.ndarray): [4] Updated orientation quaternion.
        """
        if sensor_id in self.sensor_positions:
            self.sensor_positions[sensor_id] = np.array(position, dtype=np.float32)
            self.sensor_orientations[sensor_id] = np.array(
                orientation, dtype=np.float32
            )

    def set_sensor_active(self, sensor_id: str, active: bool) -> None:
        """
        Enable or disable dust effects for a sensor.

        Args:
            sensor_id (str): Sensor identifier.
            active (bool): True to enable effects, False to disable.
        """
        if sensor_id in self.sensor_active:
            self.sensor_active[sensor_id] = active

    def apply_lidar_effects(
        self,
        sensor_id: str,
        points: np.ndarray,
        intensities: Optional[np.ndarray] = None,
        particles: Optional[List[DustParticle]] = None,
    ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        Apply dust scatter and noise to LiDAR point cloud.

        Effects applied:
        - Random point dropout in high dust regions
        - Range noise proportional to dust density
        - False positive points from dust reflections
        - Intensity reduction due to scattering

        Args:
            sensor_id (str): LiDAR sensor identifier.
            points (np.ndarray): [N, 3] Point cloud coordinates.
            intensities (Optional[np.ndarray]): [N] Point intensities.
            particles (Optional[List[DustParticle]]): Active dust particles.

        Returns:
            Tuple[np.ndarray, Optional[np.ndarray]]: Modified points and intensities.
        """
        if not self.sensor_active.get(sensor_id, False) or particles is None:
            return points, intensities

        if len(particles) == 0 or len(points) == 0:
            return points, intensities

        # Calculate dust density around sensor
        sensor_pos = self.sensor_positions.get(sensor_id)
        if sensor_pos is None:
            return points, intensities

        dust_density = self._calculate_local_dust_density(sensor_pos, particles)

        if dust_density < 1e-6:
            return points, intensities

        # Scale effects by dust density
        effect_strength = min(dust_density * self.lidar_noise_scale, 1.0)

        # Apply point dropout
        dropout_rate = effect_strength * 0.15  # Up to 15% dropout
        mask = np.random.random(len(points)) > dropout_rate
        filtered_points = points[mask]

        # Add range noise
        if len(filtered_points) > 0:
            noise_std = effect_strength * 0.02  # Up to 2cm noise
            range_noise = np.random.normal(0, noise_std, filtered_points.shape)
            filtered_points = filtered_points + range_noise

        # Reduce intensities
        if intensities is not None:
            filtered_intensities = intensities[mask]
            intensity_reduction = 1.0 - (effect_strength * 0.3)  # Up to 30% reduction
            filtered_intensities = filtered_intensities * intensity_reduction
        else:
            filtered_intensities = None

        # Add false positive dust reflections (rare)
        if effect_strength > 0.5 and len(particles) > 10:
            num_false_positives = min(int(effect_strength * 5), 10)
            nearby_particles = self._get_nearby_particles(
                sensor_pos, particles, radius=5.0
            )

            if len(nearby_particles) > 0:
                false_points = self._generate_false_positive_points(
                    sensor_pos, nearby_particles, num_false_positives
                )
                filtered_points = np.vstack([filtered_points, false_points])

                if filtered_intensities is not None:
                    false_intensities = np.random.uniform(0.1, 0.3, num_false_positives)
                    filtered_intensities = np.concatenate(
                        [filtered_intensities, false_intensities]
                    )

        return filtered_points, filtered_intensities

    def apply_camera_effects(
        self,
        sensor_id: str,
        image: np.ndarray,
        particles: Optional[List[DustParticle]] = None,
        dt: float = 0.016,
    ) -> np.ndarray:
        """
        Apply dust haze and quality degradation to camera image.

        Effects applied:
        - Atmospheric haze/fog based on dust density
        - Contrast reduction
        - Color shift toward tan/grey (lunar regolith color)
        - Edge softening (blur)

        Args:
            sensor_id (str): Camera sensor identifier.
            image (np.ndarray): [H, W, 3] RGB image (0-255 or 0-1).
            particles (Optional[List[DustParticle]]): Active dust particles.
            dt (float): Time step for temporal accumulation.

        Returns:
            np.ndarray: Modified image with dust effects.
        """
        if not self.sensor_active.get(sensor_id, False) or particles is None:
            return image

        if len(particles) == 0 or image is None:
            return image

        # Calculate dust density in front of camera
        sensor_pos = self.sensor_positions.get(sensor_id)
        if sensor_pos is None:
            return image

        # Get camera viewing direction
        orientation = self.sensor_orientations.get(sensor_id)
        if orientation is not None:
            view_direction = self._quaternion_to_forward_vector(orientation)
        else:
            view_direction = np.array([1, 0, 0])  # Default forward

        # Sample dust density along view frustum
        dust_density = self._calculate_view_frustum_density(
            sensor_pos, view_direction, particles
        )

        if dust_density < 1e-6:
            return image

        # Scale effects
        effect_strength = min(dust_density * self.camera_haze_scale, 1.0)

        # Temporal accumulation for smooth transitions
        current_haze = self._camera_haze_accumulation.get(sensor_id, 0.0)
        target_haze = effect_strength
        haze = current_haze + (target_haze - current_haze) * 0.1  # Smooth transition
        self._camera_haze_accumulation[sensor_id] = haze

        # Normalize image to 0-1 if needed
        if image.dtype == np.uint8:
            img = image.astype(np.float32) / 255.0
        else:
            img = image.copy()

        # Apply atmospheric haze
        haze_color = np.array([0.76, 0.70, 0.60])  # Lunar regolith tan color
        haze_mask = haze * 0.7  # Max 70% haze
        img = img * (1 - haze_mask) + haze_color * haze_mask

        # Reduce contrast
        contrast_factor = 1.0 - (haze * 0.4)  # Up to 40% contrast reduction
        img = (img - 0.5) * contrast_factor + 0.5

        # Clip to valid range
        img = np.clip(img, 0, 1)

        # Convert back to original format
        if image.dtype == np.uint8:
            return (img * 255).astype(np.uint8)
        return img

    def apply_imu_effects(
        self,
        sensor_id: str,
        linear_accel: np.ndarray,
        angular_vel: np.ndarray,
        particles: Optional[List[DustParticle]] = None,
        robot_velocity: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Apply dust impact vibrations to IMU readings.

        Effects applied:
        - High-frequency vibration noise from dust particle impacts
        - Amplitude scales with robot velocity and dust density
        - Temporal correlation for realistic vibration patterns

        Args:
            sensor_id (str): IMU sensor identifier.
            linear_accel (np.ndarray): [3] Linear acceleration reading.
            angular_vel (np.ndarray): [3] Angular velocity reading.
            particles (Optional[List[DustParticle]]): Active dust particles.
            robot_velocity (Optional[np.ndarray]): [3] Robot linear velocity.

        Returns:
            Tuple[np.ndarray, np.ndarray]: Modified linear acceleration and angular velocity.
        """
        if not self.sensor_active.get(sensor_id, False) or particles is None:
            return linear_accel, angular_vel

        if len(particles) == 0:
            return linear_accel, angular_vel

        # Calculate dust density around robot/IMU
        sensor_pos = self.sensor_positions.get(sensor_id)
        if sensor_pos is None:
            return linear_accel, angular_vel

        dust_density = self._calculate_local_dust_density(sensor_pos, particles)

        if dust_density < 1e-6:
            return linear_accel, angular_vel

        # Scale effect by robot velocity (more dust kicked up at speed)
        velocity_scale = 1.0
        if robot_velocity is not None:
            speed = np.linalg.norm(robot_velocity)
            velocity_scale = min(1.0 + speed * 0.5, 3.0)  # 1x to 3x scaling

        effect_strength = min(
            dust_density * self.imu_vibration_scale * velocity_scale, 1.0
        )

        # Generate vibration noise
        vibration_freq = 50.0  # Hz - dust impacts are high frequency
        vibration_amplitude = effect_strength * 0.5  # Up to 0.5 m/s² or rad/s

        # Create correlated noise (not purely random)
        noise = np.random.normal(0, vibration_amplitude, 3)

        # Add temporal correlation
        history = self._imu_vibration_history.get(sensor_id, [])
        if len(history) > 0:
            # Blend with previous noise for temporal coherence
            noise = 0.7 * noise + 0.3 * history[-1]

        # Update history
        history.append(noise)
        if len(history) > self._history_max_len:
            history.pop(0)
        self._imu_vibration_history[sensor_id] = history

        # Apply to readings
        modified_accel = linear_accel + noise * 0.8  # 80% to accel
        modified_angular = angular_vel + noise * 0.3  # 30% to angular vel

        return modified_accel, modified_angular

    def _calculate_local_dust_density(
        self, position: np.ndarray, particles: List[DustParticle]
    ) -> float:
        """
        Calculate dust particle density around a position.

        Args:
            position (np.ndarray): [3] Center position.
            particles (List[DustParticle]): Active dust particles.

        Returns:
            float: Normalized dust density (0-1+).
        """
        if len(particles) == 0:
            return 0.0

        # Count particles within radius
        radius = 3.0  # meters
        count = 0

        for particle in particles:
            distance = np.linalg.norm(particle.position - position)
            if distance < radius:
                # Weight by inverse distance
                weight = 1.0 - (distance / radius)
                count += weight

        # Normalize by expected max (rough estimate)
        max_expected = 100.0
        density = min(count / max_expected, 2.0)  # Cap at 2.0

        return density

    def _calculate_view_frustum_density(
        self,
        sensor_pos: np.ndarray,
        view_direction: np.ndarray,
        particles: List[DustParticle],
        frustum_length: float = 10.0,
        frustum_angle: float = np.pi / 6,
    ) -> float:
        """
        Calculate dust density within camera view frustum.

        Args:
            sensor_pos (np.ndarray): [3] Camera position.
            view_direction (np.ndarray): [3] Normalized view direction.
            particles (List[DustParticle]): Active dust particles.
            frustum_length (float): Length of view frustum.
            frustum_angle (float): Half-angle of view cone.

        Returns:
            float: Normalized dust density in view (0-1+).
        """
        if len(particles) == 0:
            return 0.0

        count = 0.0

        for particle in particles:
            # Vector from sensor to particle
            to_particle = particle.position - sensor_pos
            distance = np.linalg.norm(to_particle)

            # Check if within frustum length
            if distance > frustum_length:
                continue

            # Check if within frustum angle
            if distance > 0.1:  # Avoid division by zero
                direction = to_particle / distance
                angle = np.arccos(np.clip(np.dot(view_direction, direction), -1, 1))

                if angle < frustum_angle:
                    # Weight by distance (closer = more visible)
                    weight = 1.0 - (distance / frustum_length)
                    count += weight

        max_expected = 50.0
        density = min(count / max_expected, 2.0)

        return density

    def _get_nearby_particles(
        self,
        position: np.ndarray,
        particles: List[DustParticle],
        radius: float = 5.0,
    ) -> List[DustParticle]:
        """
        Get particles within radius of position.

        Args:
            position (np.ndarray): [3] Center position.
            particles (List[DustParticle]): All particles.
            radius (float): Search radius.

        Returns:
            List[DustParticle]: Nearby particles.
        """
        nearby = []
        for particle in particles:
            distance = np.linalg.norm(particle.position - position)
            if distance < radius:
                nearby.append(particle)
        return nearby

    def _generate_false_positive_points(
        self,
        sensor_pos: np.ndarray,
        nearby_particles: List[DustParticle],
        num_points: int,
    ) -> np.ndarray:
        """
        Generate false positive LiDAR returns from dust reflections.

        Args:
            sensor_pos (np.ndarray): [3] Sensor position.
            nearby_particles (List[DustParticle]): Nearby dust particles.
            num_points (int): Number of false points to generate.

        Returns:
            np.ndarray: [num_points, 3] False positive points.
        """
        false_points = []

        for _ in range(num_points):
            # Pick random nearby particle
            if len(nearby_particles) == 0:
                break

            particle = nearby_particles[np.random.randint(len(nearby_particles))]

            # Generate point along ray to particle with some offset
            direction = particle.position - sensor_pos
            distance = np.linalg.norm(direction)

            if distance > 0.1:
                direction = direction / distance
                # Place point at random distance along ray
                false_distance = np.random.uniform(0.5, distance)
                point = sensor_pos + direction * false_distance

                # Add noise
                point += np.random.normal(0, 0.05, 3)
                false_points.append(point)

        if len(false_points) == 0:
            return np.array([]).reshape(0, 3)

        return np.array(false_points)

    def _quaternion_to_forward_vector(self, quaternion: np.ndarray) -> np.ndarray:
        """
        Convert quaternion to forward vector.

        Args:
            quaternion (np.ndarray): [4] Quaternion [x, y, z, w].

        Returns:
            np.ndarray: [3] Forward direction vector.
        """
        # Standard forward vector is +X in many robotics conventions
        forward = np.array([1, 0, 0])

        # Rotate by quaternion
        try:
            r = R.from_quat(quaternion)  # scipy uses [x, y, z, w]
            return r.apply(forward)
        except:
            return forward

    def get_sensor_visibility(
        self, sensor_id: str, particles: List[DustParticle]
    ) -> float:
        """
        Calculate visibility score for a sensor (0-1, 1 = clear).

        Args:
            sensor_id (str): Sensor identifier.
            particles (List[DustParticle]): Active dust particles.

        Returns:
            float: Visibility score (0.0-1.0).
        """
        if not self.sensor_active.get(sensor_id, False) or len(particles) == 0:
            return 1.0

        sensor_pos = self.sensor_positions.get(sensor_id)
        if sensor_pos is None:
            return 1.0

        density = self._calculate_local_dust_density(sensor_pos, particles)
        visibility = max(0.0, 1.0 - (density * 0.5))

        return visibility

    def reset(self) -> None:
        """
        Reset all sensor effect accumulations.

        Called when simulation resets.
        """
        for sensor_id in self._camera_haze_accumulation:
            self._camera_haze_accumulation[sensor_id] = 0.0

        for sensor_id in self._imu_vibration_history:
            self._imu_vibration_history[sensor_id] = []
