__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

import dataclasses
from typing import Any, Union, List


@dataclasses.dataclass
class CraterGeneratorConf:
    profiles_path: str = dataclasses.field(default_factory=str)
    min_xy_ratio: float = dataclasses.field(default_factory=float)
    max_xy_ratio: float = dataclasses.field(default_factory=float)
    resolution: float = dataclasses.field(default_factory=float)
    pad_size: int = dataclasses.field(default_factory=int)
    random_rotation: bool = dataclasses.field(default_factory=bool)
    z_scale: float = dataclasses.field(default_factory=float)
    seed: int = dataclasses.field(default_factory=int)

    def __post_init__(self):
        print(self.__dict__)
        assert type(self.profiles_path) is str, "profile_path must be a string"
        assert type(self.min_xy_ratio) is float, "min_xy_ratio must be a float"
        assert type(self.max_xy_ratio) is float, "max_xy_ratio must be a float"
        assert type(self.resolution) is float, "resolution must be a float"
        assert type(self.pad_size) is int, "pad_size must be an integer"
        assert type(self.random_rotation) is bool, "random_rotation must be a boolean"
        assert type(self.z_scale) is float, "z_scale must be a float"
        assert type(self.seed) is int, "seed must be an integer"

        assert self.min_xy_ratio <= self.max_xy_ratio, (
            "min_xy_ratio must be smaller than max_xy_ratio"
        )
        assert self.min_xy_ratio > 0, "min_xy_ratio must be greater than 0"
        assert self.max_xy_ratio > 0, "max_xy_ratio must be greater than 0"
        assert self.min_xy_ratio <= 1, "min_xy_ratio must be smaller than 1"
        assert self.max_xy_ratio <= 1, "max_xy_ratio must be smaller than 1"
        assert self.resolution > 0, "resolution must be greater than 0"
        assert self.pad_size >= 0, "pad_size must be greater or equal to 0"
        assert self.z_scale > 0, "z_scale must be greater than 0"


@dataclasses.dataclass
class DustPhysicsConf:
    """
    Dust physics simulation parameters for lunar regolith dust dynamics.

    Implements particle-based dust emitters at wheel-terrain contact points
    with ballistic trajectories, drag, and empirical cohesion effects.

    Args:
        enable (bool): Enable dust physics simulation.
        emission_rate (int): Particles emitted per second per wheel.
        particle_lifetime (float): Maximum lifetime of particles in seconds.
        particle_size_min (float): Minimum particle size in meters (fine dust).
        particle_size_max (float): Maximum particle size in meters (coarse).
        particle_density (float): Particle density in kg/m^3 (lunar regolith).
        gravity (list): Gravity vector [x, y, z] in m/s^2.
        emission_velocity_threshold (float): Minimum slip velocity to trigger emission (m/s).
        emission_force_threshold (float): Minimum contact force to trigger emission (N).
        drag_coefficient (float): Drag coefficient for particle aerodynamics.
        cohesion_strength (float): Empirical cohesion factor for lunar dust.
        settling_velocity (float): Velocity threshold for particle settling (m/s).
        max_particles (int): Maximum number of particles in the system.
        particle_color (list): RGB color values [0-1] for dust visualization.
        particle_opacity (float): Opacity value [0-1] for dust particles.
        publish_dust_density (bool): Enable ROS2 dust density topic publication.
        dust_density_topic (str): ROS2 topic name for dust density map.
        dust_visibility_topic (str): ROS2 topic name for visibility estimates.
        publish_rate (float): ROS2 publication rate in Hz.
    """

    enable: bool = False
    emission_rate: int = 1000
    particle_lifetime: float = 2.0
    particle_size_min: float = 0.001
    particle_size_max: float = 0.1
    particle_density: float = 1500.0
    gravity: List[float] = dataclasses.field(default_factory=lambda: [0.0, 0.0, -1.62])
    emission_velocity_threshold: float = 0.1
    emission_force_threshold: float = 50.0
    drag_coefficient: float = 0.47
    cohesion_strength: float = 0.01
    settling_velocity: float = 0.5
    max_particles: int = 50000
    particle_color: List[float] = dataclasses.field(
        default_factory=lambda: [0.7, 0.65, 0.6]
    )
    particle_opacity: float = 0.8
    publish_dust_density: bool = True
    dust_density_topic: str = "/dust/density_map"
    dust_visibility_topic: str = "/dust/visibility"
    publish_rate: float = 10.0

    # Sensor degradation effects
    enable_sensor_effects: bool = True
    lidar_dust_noise_scale: float = 1.0
    camera_dust_haze_scale: float = 1.0
    imu_dust_vibration_scale: float = 1.0

    def __post_init__(self):
        assert type(self.enable) is bool, "enable must be a boolean"
        assert type(self.emission_rate) is int, "emission_rate must be an integer"
        assert type(self.particle_lifetime) is float, (
            "particle_lifetime must be a float"
        )
        assert type(self.particle_size_min) is float, (
            "particle_size_min must be a float"
        )
        assert type(self.particle_size_max) is float, (
            "particle_size_max must be a float"
        )
        assert type(self.particle_density) is float, "particle_density must be a float"
        assert type(self.emission_velocity_threshold) is float, (
            "emission_velocity_threshold must be a float"
        )
        assert type(self.emission_force_threshold) is float, (
            "emission_force_threshold must be a float"
        )
        assert type(self.drag_coefficient) is float, "drag_coefficient must be a float"
        assert type(self.cohesion_strength) is float, (
            "cohesion_strength must be a float"
        )
        assert type(self.settling_velocity) is float, (
            "settling_velocity must be a float"
        )
        assert type(self.max_particles) is int, "max_particles must be an integer"
        assert type(self.particle_opacity) is float, "particle_opacity must be a float"
        assert type(self.publish_dust_density) is bool, (
            "publish_dust_density must be a boolean"
        )
        assert type(self.dust_density_topic) is str, (
            "dust_density_topic must be a string"
        )
        assert type(self.dust_visibility_topic) is str, (
            "dust_visibility_topic must be a string"
        )
        assert type(self.publish_rate) is float, "publish_rate must be a float"

        # Sensor effects validation
        assert type(self.enable_sensor_effects) is bool, (
            "enable_sensor_effects must be a boolean"
        )
        assert type(self.lidar_dust_noise_scale) is float, (
            "lidar_dust_noise_scale must be a float"
        )
        assert type(self.camera_dust_haze_scale) is float, (
            "camera_dust_haze_scale must be a float"
        )
        assert type(self.imu_dust_vibration_scale) is float, (
            "imu_dust_vibration_scale must be a float"
        )

        assert self.emission_rate > 0, "emission_rate must be greater than 0"
        assert self.particle_lifetime > 0, "particle_lifetime must be greater than 0"
        assert self.particle_size_min > 0, "particle_size_min must be greater than 0"
        assert self.particle_size_max > self.particle_size_min, (
            "particle_size_max must be greater than particle_size_min"
        )
        assert self.particle_density > 0, "particle_density must be greater than 0"
        assert len(self.gravity) == 3, "gravity must be a list of length 3"
        assert self.emission_velocity_threshold >= 0, (
            "emission_velocity_threshold must be greater than or equal to 0"
        )
        assert self.emission_force_threshold >= 0, (
            "emission_force_threshold must be greater than or equal to 0"
        )
        assert self.drag_coefficient > 0, "drag_coefficient must be greater than 0"
        assert self.cohesion_strength >= 0, (
            "cohesion_strength must be greater than or equal to 0"
        )
        assert self.settling_velocity > 0, "settling_velocity must be greater than 0"
        assert self.max_particles > 0, "max_particles must be greater than 0"
        assert len(self.particle_color) == 3, (
            "particle_color must be a list of length 3"
        )
        assert all(0 <= c <= 1 for c in self.particle_color), (
            "particle_color values must be between 0 and 1"
        )
        assert 0 <= self.particle_opacity <= 1, (
            "particle_opacity must be between 0 and 1"
        )
        assert self.publish_rate > 0, "publish_rate must be greater than 0"

        # Sensor effects range validation
        assert self.lidar_dust_noise_scale >= 0, (
            "lidar_dust_noise_scale must be greater than or equal to 0"
        )
        assert self.camera_dust_haze_scale >= 0, (
            "camera_dust_haze_scale must be greater than or equal to 0"
        )
        assert self.imu_dust_vibration_scale >= 0, (
            "imu_dust_vibration_scale must be greater than or equal to 0"
        )


@dataclasses.dataclass
class CraterDistributionConf:
    x_size: float = dataclasses.field(default_factory=float)
    y_size: float = dataclasses.field(default_factory=float)
    densities: list = dataclasses.field(default_factory=list)
    radius: list = dataclasses.field(default_factory=list)
    num_repeat: int = dataclasses.field(default_factory=int)
    seed: int = dataclasses.field(default_factory=int)

    def __post_init__(self):
        assert type(self.x_size) is float, "x_size must be a float"
        assert type(self.y_size) is float, "y_size must be a float"
        assert type(self.num_repeat) is int, "num_repeat must be an integer"
        assert type(self.seed) is int, "seed must be an integer"

        assert self.x_size > 0, "x_size must be greater than 0"
        assert self.y_size > 0, "y_size must be greater than 0"
        assert len(self.densities) == len(self.radius), (
            "densities and radius must have the same length"
        )
        assert self.num_repeat >= 0, "num_repeat must be greater or equal to 0"


@dataclasses.dataclass
class BaseTerrainGeneratorConf:
    x_size: float = dataclasses.field(default_factory=float)
    y_size: float = dataclasses.field(default_factory=float)
    resolution: float = dataclasses.field(default_factory=float)
    max_elevation: float = dataclasses.field(default_factory=float)
    min_elevation: float = dataclasses.field(default_factory=float)
    seed: int = dataclasses.field(default_factory=int)
    z_scale: float = dataclasses.field(default_factory=float)

    def __post_init__(self):
        assert type(self.x_size) is float, "x_size must be a float"
        assert type(self.y_size) is float, "y_size must be a float"
        assert type(self.resolution) is float, "resolution must be a float"
        assert type(self.max_elevation) is float, "max_elevation must be a float"
        assert type(self.min_elevation) is float, "min_elevation must be a float"
        assert type(self.seed) is int, "seed must be an integer"
        assert type(self.z_scale) is float, "z_scale must be a float"

        assert self.x_size > 0, "x_size must be greater than 0"
        assert self.y_size > 0, "y_size must be greater than 0"
        assert self.resolution > 0, "resolution must be greater than 0"
        assert self.max_elevation > self.min_elevation, (
            "max_elevation must be greater than min_elevation"
        )
        assert self.z_scale > 0, "z_scale must be greater than 0"


@dataclasses.dataclass
class FootprintConf:
    """
    Footprint dimension parameters.
    We use FLU (Front, Left, Up) coordinate system for the footprint.
    Args:
        width (float): Width of the footprint.
        height (float): Height of the footprint.
        shape (str): Shape of the footprint.
    """

    width: float = 0.5
    height: float = 0.25
    shape: str = "rectangle"

    def __post_init__(self):
        assert type(self.width) is float, "wheel_width must be a float"
        assert type(self.height) is float, "wheel_radius must be a float"
        assert type(self.shape) is str, "shape must be a string"
        assert self.width > 0, "wheel_width must be greater than 0"
        assert self.height > 0, "wheel_radius must be greater than 0"


@dataclasses.dataclass
class DeformConstrainConf:
    """
    Deformation constrain parameters.
    Args:
        x_deform_offset (float): X offset betweem deformation center and contact point.
        y_deform_offset (float): Y offset betweem deformation center and contact point.
        deform_decay_ratio (float): Decay ratio of the deformation.
    """

    x_deform_offset: float = 0.0
    y_deform_offset: float = 0.0
    deform_decay_ratio: float = 0.01

    def __post_init__(self):
        assert type(self.x_deform_offset) is float, "deform_offset must be a float"
        assert type(self.y_deform_offset) is float, "deform_offset must be a float"
        assert type(self.deform_decay_ratio) is float, (
            "deform_decay_ratio must be a float"
        )
        assert self.deform_decay_ratio > 0, "deform_decay_ratio must be greater than 0"


@dataclasses.dataclass
class BoundaryDistributionConf:
    """
    Boundary distribution parameters.
    Args:
        distribution (str): Distribution of the boundary.
        angle_of_repose (float): Angle of repose of the boundary (only used for trapezoidal distribution)
    """

    distribution: str = "uniform"
    angle_of_repose: float = 1.047

    def __post_init__(self):
        assert type(self.distribution) is str, "distribution must be a string"
        assert type(self.angle_of_repose) is float, "angle_of_repose must be a float"
        assert self.angle_of_repose > 0, "angle_of_repose must be greater than 0"


@dataclasses.dataclass
class DepthDistributionConf:
    """
    Deformation depth distribution parameters.
    Args:
        distribution (str): Distribution of the force.
        wave_frequency (float): Frequency of the wave. Under no slip condition, this is num_grouser/pi
    """

    distribution: str = "uniform"
    wave_frequency: float = 1.0

    def __post_init__(self):
        assert type(self.distribution) is str, "distribution must be a string"
        assert self.wave_frequency > 0, "wave_frequency must be greater than 0"


@dataclasses.dataclass
class ForceDepthRegressionConf:
    """
    Force depth regression parameters.
    For now, linear regression.
    Args:
        amplitude_slope (float): Slope of the amplitude.
        amplitude_intercept (float): Intercept of the amplitude.
        mean_slope (float): Slope of the mean.
        mean_intercept (float): Intercept of the mean.
    """

    amplitude_slope: float = 1.0
    amplitude_intercept: float = 0.0
    mean_slope: float = 1.0
    mean_intercept: float = 0.0

    def __post_init__(self):
        assert type(self.amplitude_slope) is float, "slope must be a float"
        assert type(self.amplitude_intercept) is float, "intercept must be a float"
        assert type(self.mean_slope) is float, "slope must be a float"
        assert type(self.mean_intercept) is float, "intercept must be a float"


@dataclasses.dataclass
class DeformationEngineConf:
    """
    Deformation engine parameters.
    Args:
        enable (bool): Enable deformation.
        delay (float): Delay time (s) for the deformation.
        terrain_resolution (float): Resolution of the terrain.
        terrain_width (float): Width of the terrain.
        terrain_height (float): Height of the terrain.
        gravity (list): Gravity vector.
        footprint (dict): Footprint parameters.
        deform_constrain (dict): Deformation constrain parameters.
        boundary_distribution (dict): Boundary distribution parameters.
        depth_distribution (dict): Deformation depth distribution parameters.
        force_depth_regression (dict): Force depth regression parameters.
        num_links (int): Total number of links = num_robot * num_target_links.
    """

    enable: bool = False
    delay: float = 1.0
    terrain_resolution: float = dataclasses.field(default_factory=float)
    terrain_width: float = dataclasses.field(default_factory=float)
    terrain_height: float = dataclasses.field(default_factory=float)
    gravity: List[float] = dataclasses.field(default_factory=list)
    footprint: FootprintConf = dataclasses.field(default_factory=dict)
    deform_constrain: DeformConstrainConf = dataclasses.field(default_factory=dict)
    boundary_distribution: BoundaryDistributionConf = dataclasses.field(
        default_factory=dict
    )
    depth_distribution: DepthDistributionConf = dataclasses.field(default_factory=dict)
    force_depth_regression: ForceDepthRegressionConf = dataclasses.field(
        default_factory=dict
    )
    num_links: int = 4

    def __post_init__(self):
        assert type(self.delay) is float, "delay must be float"
        assert type(self.terrain_resolution) is float, (
            "terrain_resolution must be a float"
        )
        assert self.delay >= 0, "render_deform_inv must be greater than or equal to 1"
        assert self.terrain_resolution > 0, "terrain_resolution must be greater than 0"
        assert self.terrain_width > 0, "terrain_width must be greater than 0"
        assert self.terrain_height > 0, "terrain_height must be greater than 0"
        assert self.num_links > 0, "num_links must be greater than 0"

        self.footprint = FootprintConf(**self.footprint)
        self.deform_constrain = DeformConstrainConf(**self.deform_constrain)
        self.boundary_distribution = BoundaryDistributionConf(
            **self.boundary_distribution
        )
        self.depth_distribution = DepthDistributionConf(**self.depth_distribution)
        self.force_depth_regression = ForceDepthRegressionConf(
            **self.force_depth_regression
        )


@dataclasses.dataclass
class MoonYardConf:
    crater_generator: CraterGeneratorConf = None
    crater_distribution: CraterDistributionConf = None
    base_terrain_generator: BaseTerrainGeneratorConf = None
    deformation_engine: DeformationEngineConf = None
    dust_physics: DustPhysicsConf = None
    is_yard: bool = dataclasses.field(default_factory=bool)
    is_lab: bool = dataclasses.field(default_factory=bool)

    def __post_init__(self):
        self.crater_generator = CraterGeneratorConf(**self.crater_generator)
        self.crater_distribution = CraterDistributionConf(**self.crater_distribution)
        self.base_terrain_generator = BaseTerrainGeneratorConf(
            **self.base_terrain_generator
        )
        self.deformation_engine = DeformationEngineConf(**self.deformation_engine)
        self.dust_physics = DustPhysicsConf(**self.dust_physics)

        assert type(self.is_yard) is bool, "is_yard must be a boolean"
        assert type(self.is_lab) is bool, "is_lab must be a boolean"


@dataclasses.dataclass
class TerrainManagerConf:
    moon_yard: MoonYardConf = None
    root_path: str = dataclasses.field(default_factory=str)
    texture_path: str = dataclasses.field(default_factory=str)
    dems_path: str = dataclasses.field(default_factory=str)
    mesh_position: tuple = dataclasses.field(default_factory=tuple)
    mesh_orientation: tuple = dataclasses.field(default_factory=tuple)
    mesh_scale: tuple = dataclasses.field(default_factory=tuple)
    sim_length: int = dataclasses.field(default_factory=int)
    sim_width: int = dataclasses.field(default_factory=int)
    resolution: float = dataclasses.field(default_factory=float)
    augmentation: bool = False

    def __post_init__(self):
        self.moon_yard = MoonYardConf(**self.moon_yard)

        assert type(self.root_path) is str, "root_path must be a string"
        assert type(self.texture_path) is str, "texture_path must be a string"
        assert type(self.dems_path) is str, "dems_path must be a string"
        assert type(self.sim_length) is float, "sim_length must be a float"
        assert type(self.sim_width) is float, "sim_width must be a float"
        assert type(self.resolution) is float, "resolution must be a float"

        assert len(self.mesh_position) == 3, "mesh_position must be a tuple of length 3"
        assert len(self.mesh_orientation) == 4, (
            "mesh_orientation must be a tuple of length 4"
        )
        assert len(self.mesh_scale) == 3, "mesh_scale must be a tuple of length 3"
        assert self.sim_length > 0, "sim_length must be greater than 0"
        assert self.sim_width > 0, "sim_width must be greater than 0"
        assert self.resolution > 0, "resolution must be greater than 0"
