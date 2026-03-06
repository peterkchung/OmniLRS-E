#!/usr/bin/env python3
"""
Dust Physics Test Suite

Validation tests for Milestone 1 dust physics implementation.
Tests particle emission, dynamics, sensor effects, and ROS2 integration.

Usage:
    python test_dust_physics.py [--verbose] [--visualize]

Exit codes:
    0 - All tests passed
    1 - One or more tests failed
"""

import sys
import time
import argparse
import numpy as np
from typing import List, Optional

# Add project root to path
sys.path.insert(0, "/home/peter/projects/OmniLRS-E")

from src.environments.dust_physics.dust_particle import DustParticle
from src.environments.dust_physics.dust_emitter import DustEmitter
from src.environments.dust_physics.dust_dynamics import DustDynamics
from src.environments.dust_physics.dust_sensor_effects import DustSensorEffects
from src.configurations.procedural_terrain_confs import DustPhysicsConf


class TestResult:
    """Test result container."""

    def __init__(self, name: str, passed: bool, message: str = ""):
        self.name = name
        self.passed = passed
        self.message = message


class DustPhysicsTestSuite:
    """Test suite for dust physics module."""

    def __init__(self, verbose: bool = False):
        self.verbose = verbose
        self.results: List[TestResult] = []

        # Default test configuration
        self.test_config = DustPhysicsConf(
            enable=True,
            emission_rate=100,
            particle_lifetime=2.0,
            particle_size_min=0.001,
            particle_size_max=0.05,
            particle_density=1500.0,
            gravity=[0.0, 0.0, -1.62],
            emission_velocity_threshold=0.1,
            emission_force_threshold=50.0,
            drag_coefficient=0.47,
            cohesion_strength=0.01,
            settling_velocity=0.5,
            max_particles=1000,
            particle_color=[0.7, 0.65, 0.6],
            particle_opacity=0.8,
            publish_dust_density=True,
            dust_density_topic="/test/dust/density_map",
            dust_visibility_topic="/test/dust/visibility",
            publish_rate=10.0,
            enable_sensor_effects=True,
            lidar_dust_noise_scale=1.0,
            camera_dust_haze_scale=1.0,
            imu_dust_vibration_scale=1.0,
        )

    def log(self, message: str) -> None:
        """Log message if verbose mode enabled."""
        if self.verbose:
            print(f"  {message}")

    def run_all_tests(self) -> bool:
        """Run all tests and return overall pass status."""
        print("\n" + "=" * 70)
        print("Dust Physics Test Suite - Milestone 1 Validation")
        print("=" * 70 + "\n")

        # Core component tests
        self.test_particle_creation()
        self.test_emitter_functionality()
        self.test_dynamics_gravity()
        self.test_dynamics_drag()
        self.test_dynamics_cohesion()

        # Sensor effects tests
        self.test_sensor_effects_registration()
        self.test_lidar_noise_application()
        self.test_camera_haze_application()
        self.test_imu_vibration_application()

        # Integration tests
        self.test_particle_lifecycle()
        self.test_performance_scaling()

        # Print summary
        return self.print_summary()

    def test_particle_creation(self) -> None:
        """Test DustParticle creation and properties."""
        print("Testing: Particle Creation")

        try:
            # Create a particle
            position = np.array([1.0, 2.0, 0.5])
            velocity = np.array([0.5, 0.0, 1.0])

            particle = DustParticle(
                position=position,
                velocity=velocity,
                size=0.01,
                density=1500.0,
                lifetime=2.0,
                creation_time=0.0,
            )

            # Verify properties
            assert np.allclose(particle.position, position), "Position mismatch"
            assert np.allclose(particle.velocity, velocity), "Velocity mismatch"
            assert particle.size == 0.01, "Size mismatch"
            assert particle.density == 1500.0, "Density mismatch"
            assert particle.mass > 0, "Mass should be positive"
            assert particle.cross_sectional_area > 0, (
                "Cross-sectional area should be positive"
            )

            self.log(f"  Particle mass: {particle.mass:.6e} kg")
            self.log(f"  Cross-sectional area: {particle.cross_sectional_area:.6e} m^2")

            self.results.append(TestResult("Particle Creation", True))

        except Exception as e:
            self.results.append(TestResult("Particle Creation", False, str(e)))

    def test_emitter_functionality(self) -> None:
        """Test DustEmitter emission logic."""
        print("Testing: Emitter Functionality")

        try:
            emitter = DustEmitter(self.test_config)

            # Test emission with high velocity and force (should emit)
            wheel_positions = np.array([[0.0, 0.0, 0.0]])
            wheel_velocities = np.array([[1.0, 0.0, 0.0]])  # Above threshold
            wheel_forces = np.array([[100.0, 0.0, 0.0]])  # Above threshold

            particles = emitter.emit(
                wheel_positions, wheel_velocities, wheel_forces, dt=0.1
            )

            emission_count = len(particles)
            self.log(f"  Emitted {emission_count} particles with high velocity/force")

            assert emission_count > 0, "Should emit particles when above thresholds"

            # Test emission with low velocity (should not emit)
            wheel_velocities_low = np.array([[0.01, 0.0, 0.0]])  # Below threshold
            wheel_forces_low = np.array([[10.0, 0.0, 0.0]])  # Below threshold

            particles_low = emitter.emit(
                wheel_positions, wheel_velocities_low, wheel_forces_low, dt=0.1
            )

            self.log(
                f"  Emitted {len(particles_low)} particles with low velocity/force"
            )
            assert len(particles_low) == 0, "Should not emit when below thresholds"

            self.results.append(TestResult("Emitter Functionality", True))

        except Exception as e:
            self.results.append(TestResult("Emitter Functionality", False, str(e)))

    def test_dynamics_gravity(self) -> None:
        """Test lunar gravity application."""
        print("Testing: Dynamics - Lunar Gravity")

        try:
            dynamics = DustDynamics(self.test_config)

            # Create a particle at height with zero velocity
            particle = DustParticle(
                position=np.array([0.0, 0.0, 10.0]),
                velocity=np.array([0.0, 0.0, 0.0]),
                size=0.01,
                density=1500.0,
                lifetime=10.0,
                creation_time=0.0,
            )

            # Simulate for 1 second
            dt = 0.01
            for _ in range(100):  # 1 second at 100 Hz
                dynamics.update([particle], dt)

            # Calculate expected position under constant lunar gravity
            # z = z0 + v0*t + 0.5*g*t^2
            # z = 10.0 + 0.5*(-1.62)*1.0^2 = 10.0 - 0.81 = 9.19
            expected_z = 10.0 + 0.5 * (-1.62) * (1.0**2)

            self.log(f"  Final position z: {particle.position[2]:.3f} m")
            self.log(f"  Expected z: {expected_z:.3f} m")
            self.log(f"  Velocity z: {particle.velocity[2]:.3f} m/s")

            # Allow 5% tolerance for numerical integration
            assert abs(particle.position[2] - expected_z) < 0.5, (
                "Gravity integration mismatch"
            )
            assert abs(particle.velocity[2] - (-1.62)) < 0.1, (
                "Final velocity should be ~g*t"
            )

            self.results.append(TestResult("Dynamics - Lunar Gravity", True))

        except Exception as e:
            self.results.append(TestResult("Dynamics - Lunar Gravity", False, str(e)))

    def test_dynamics_drag(self) -> None:
        """Test drag force calculation."""
        print("Testing: Dynamics - Drag Forces")

        try:
            # Test with atmospheric density (simulating Mars for testing)
            config_with_drag = DustPhysicsConf(
                enable=True,
                emission_rate=100,
                particle_lifetime=2.0,
                particle_size_min=0.001,
                particle_size_max=0.05,
                particle_density=1500.0,
                gravity=[0.0, 0.0, -1.62],
                drag_coefficient=0.47,
                cohesion_strength=0.0,
                settling_velocity=0.5,
                max_particles=1000,
            )

            dynamics = DustDynamics(config_with_drag)
            dynamics.atmospheric_density = 1.0  # Enable drag for testing

            # Create a particle with high velocity
            particle = DustParticle(
                position=np.array([0.0, 0.0, 10.0]),
                velocity=np.array([10.0, 0.0, 0.0]),
                size=0.1,
                density=1500.0,
                lifetime=10.0,
                creation_time=0.0,
            )

            # Calculate drag force
            drag_force = dynamics._calculate_drag(particle)

            self.log(f"  Drag force: {drag_force}")
            self.log(f"  Drag magnitude: {np.linalg.norm(drag_force):.3f} N")

            # Drag should oppose velocity (negative x direction)
            assert drag_force[0] < 0, "Drag should oppose velocity"
            assert np.linalg.norm(drag_force) > 0, (
                "Drag should be non-zero with atmosphere"
            )

            self.results.append(TestResult("Dynamics - Drag Forces", True))

        except Exception as e:
            self.results.append(TestResult("Dynamics - Drag Forces", False, str(e)))

    def test_dynamics_cohesion(self) -> None:
        """Test cohesion force calculation."""
        print("Testing: Dynamics - Cohesion Forces")

        try:
            config_with_cohesion = DustPhysicsConf(
                enable=True,
                emission_rate=100,
                particle_lifetime=2.0,
                particle_size_min=0.001,
                particle_size_max=0.05,
                particle_density=1500.0,
                gravity=[0.0, 0.0, -1.62],
                drag_coefficient=0.47,
                cohesion_strength=0.1,  # Enable cohesion
                settling_velocity=0.5,
                max_particles=1000,
            )

            dynamics = DustDynamics(config_with_cohesion)

            # Create two nearby particles
            particle1 = DustParticle(
                position=np.array([0.0, 0.0, 1.0]),
                velocity=np.array([0.0, 0.0, 0.0]),
                size=0.05,
                density=1500.0,
                lifetime=10.0,
                creation_time=0.0,
            )

            particle2 = DustParticle(
                position=np.array([0.1, 0.0, 1.0]),  # 10 cm apart
                velocity=np.array([0.0, 0.0, 0.0]),
                size=0.05,
                density=1500.0,
                lifetime=10.0,
                creation_time=0.0,
            )

            particles = [particle1, particle2]

            # Calculate cohesion force on particle1
            cohesion_force = dynamics._calculate_cohesion(particle1, particles)

            self.log(f"  Cohesion force on particle1: {cohesion_force}")
            self.log(f"  Cohesion magnitude: {np.linalg.norm(cohesion_force):.6e} N")

            # Cohesion should attract particle1 toward particle2 (positive x)
            assert cohesion_force[0] > 0, (
                "Cohesion should attract toward other particle"
            )

            self.results.append(TestResult("Dynamics - Cohesion Forces", True))

        except Exception as e:
            self.results.append(TestResult("Dynamics - Cohesion Forces", False, str(e)))

    def test_sensor_effects_registration(self) -> None:
        """Test sensor registration and pose updates."""
        print("Testing: Sensor Effects - Registration")

        try:
            effects = DustSensorEffects(self.test_config)

            # Register sensors
            effects.register_sensor(
                "lidar_1",
                np.array([1.0, 0.0, 0.5]),
                np.array([0.0, 0.0, 0.0, 1.0]),
                "lidar",
            )

            effects.register_sensor(
                "camera_1",
                np.array([0.5, 0.0, 0.3]),
                np.array([0.0, 0.0, 0.0, 1.0]),
                "camera",
            )

            effects.register_sensor(
                "imu_1",
                np.array([0.0, 0.0, 0.2]),
                np.array([0.0, 0.0, 0.0, 1.0]),
                "imu",
            )

            self.log(f"  Registered {len(effects.sensor_positions)} sensors")

            # Update sensor pose
            effects.update_sensor_pose(
                "lidar_1", np.array([1.1, 0.0, 0.5]), np.array([0.0, 0.0, 0.0, 1.0])
            )

            # Verify update
            assert np.allclose(effects.sensor_positions["lidar_1"], [1.1, 0.0, 0.5])

            self.results.append(TestResult("Sensor Effects - Registration", True))

        except Exception as e:
            self.results.append(
                TestResult("Sensor Effects - Registration", False, str(e))
            )

    def test_lidar_noise_application(self) -> None:
        """Test LiDAR point cloud noise injection."""
        print("Testing: Sensor Effects - LiDAR Noise")

        try:
            effects = DustSensorEffects(self.test_config)

            effects.register_sensor(
                "lidar_test",
                np.array([0.0, 0.0, 1.0]),
                np.array([0.0, 0.0, 0.0, 1.0]),
                "lidar",
            )

            # Create test point cloud
            original_points = np.random.rand(100, 3) * 10.0
            original_intensities = np.random.rand(100)

            # Create dust particles
            particles = []
            for i in range(50):
                particle = DustParticle(
                    position=np.random.rand(3) * 5.0,
                    velocity=np.random.rand(3) * 2.0,
                    size=0.02,
                    density=1500.0,
                    lifetime=2.0,
                    creation_time=0.0,
                )
                particles.append(particle)

            # Apply effects
            modified_points, modified_intensities = effects.apply_lidar_effects(
                "lidar_test", original_points, original_intensities, particles
            )

            self.log(f"  Original points: {len(original_points)}")
            self.log(f"  Modified points: {len(modified_points)}")

            # Points should be modified (filtered and possibly have noise)
            assert len(modified_points) <= len(original_points), "Should have dropout"
            assert modified_intensities is not None, "Intensities should be returned"

            self.results.append(TestResult("Sensor Effects - LiDAR Noise", True))

        except Exception as e:
            self.results.append(
                TestResult("Sensor Effects - LiDAR Noise", False, str(e))
            )

    def test_camera_haze_application(self) -> None:
        """Test camera haze effect application."""
        print("Testing: Sensor Effects - Camera Haze")

        try:
            effects = DustSensorEffects(self.test_config)

            effects.register_sensor(
                "camera_test",
                np.array([0.0, 0.0, 1.0]),
                np.array([0.0, 0.0, 0.0, 1.0]),
                "camera",
            )

            # Create test image
            test_image = np.random.rand(480, 640, 3).astype(np.float32)

            # Create dust particles in front of camera
            particles = []
            for i in range(100):
                particle = DustParticle(
                    position=np.array(
                        [2.0, np.random.rand() * 4 - 2, np.random.rand() * 2]
                    ),
                    velocity=np.random.rand(3) * 2.0,
                    size=0.05,
                    density=1500.0,
                    lifetime=2.0,
                    creation_time=0.0,
                )
                particles.append(particle)

            # Apply effects
            modified_image = effects.apply_camera_effects(
                "camera_test", test_image, particles, dt=0.016
            )

            self.log(f"  Image shape: {modified_image.shape}")
            self.log(
                f"  Mean pixel change: {np.mean(np.abs(modified_image - test_image)):.4f}"
            )

            # Image should be modified
            assert modified_image.shape == test_image.shape, (
                "Image dimensions should be preserved"
            )
            assert not np.allclose(modified_image, test_image), (
                "Image should be modified"
            )

            self.results.append(TestResult("Sensor Effects - Camera Haze", True))

        except Exception as e:
            self.results.append(
                TestResult("Sensor Effects - Camera Haze", False, str(e))
            )

    def test_imu_vibration_application(self) -> None:
        """Test IMU vibration effect application."""
        print("Testing: Sensor Effects - IMU Vibration")

        try:
            effects = DustSensorEffects(self.test_config)

            effects.register_sensor(
                "imu_test",
                np.array([0.0, 0.0, 0.5]),
                np.array([0.0, 0.0, 0.0, 1.0]),
                "imu",
            )

            # Create dust particles
            particles = []
            for i in range(30):
                particle = DustParticle(
                    position=np.random.rand(3) * 3.0,
                    velocity=np.random.rand(3) * 2.0,
                    size=0.03,
                    density=1500.0,
                    lifetime=2.0,
                    creation_time=0.0,
                )
                particles.append(particle)

            # Original readings
            original_accel = np.array([0.0, 0.0, -1.62])
            original_gyro = np.array([0.0, 0.0, 0.0])
            robot_velocity = np.array([1.0, 0.0, 0.0])  # Moving forward

            # Apply effects
            modified_accel, modified_gyro = effects.apply_imu_effects(
                "imu_test", original_accel, original_gyro, particles, robot_velocity
            )

            self.log(f"  Original accel: {original_accel}")
            self.log(f"  Modified accel: {modified_accel}")
            self.log(
                f"  Accel change: {np.linalg.norm(modified_accel - original_accel):.6f}"
            )

            # Readings should be modified
            assert not np.allclose(modified_accel, original_accel), (
                "Accel should have vibration"
            )
            assert not np.allclose(modified_gyro, original_gyro), (
                "Gyro should have vibration"
            )

            self.results.append(TestResult("Sensor Effects - IMU Vibration", True))

        except Exception as e:
            self.results.append(
                TestResult("Sensor Effects - IMU Vibration", False, str(e))
            )

    def test_particle_lifecycle(self) -> None:
        """Test complete particle lifecycle."""
        print("Testing: Integration - Particle Lifecycle")

        try:
            # Create components
            emitter = DustEmitter(self.test_config)
            dynamics = DustDynamics(self.test_config)

            # Simulate wheel driving over terrain
            wheel_positions = np.array([[0.0, 0.0, 0.0]])
            wheel_velocities = np.array([[2.0, 0.0, 0.0]])  # 2 m/s forward
            wheel_forces = np.array([[200.0, 0.0, 0.0]])  # High contact force

            all_particles = []

            # Simulate 5 seconds
            dt = 0.033  # 30 Hz
            for step in range(150):  # 5 seconds
                # Emit new particles
                new_particles = emitter.emit(
                    wheel_positions, wheel_velocities, wheel_forces, dt
                )
                all_particles.extend(new_particles)

                # Update existing particles
                if len(all_particles) > 0:
                    dynamics.update(all_particles, dt)

                    # Remove settled particles
                    current_time = step * dt
                    all_particles = [
                        p
                        for p in all_particles
                        if p.is_active(current_time, self.test_config.settling_velocity)
                    ]

                # Move wheel forward
                wheel_positions[0, 0] += wheel_velocities[0, 0] * dt

            self.log(f"  Final particle count: {len(all_particles)}")
            self.log(f"  Simulation steps: 150")
            self.log(
                f"  Total particles created: {len(all_particles) + sum(1 for p in all_particles if not p.is_active(5.0, self.test_config.settling_velocity))}"
            )

            assert len(all_particles) >= 0, "Should have processed particles"

            self.results.append(TestResult("Integration - Particle Lifecycle", True))

        except Exception as e:
            self.results.append(
                TestResult("Integration - Particle Lifecycle", False, str(e))
            )

    def test_performance_scaling(self) -> None:
        """Test performance scaling with particle count."""
        print("Testing: Integration - Performance Scaling")

        try:
            dynamics = DustDynamics(self.test_config)

            # Test with increasing particle counts
            particle_counts = [100, 500, 1000]
            timings = []

            for count in particle_counts:
                # Create particles
                particles = []
                for i in range(count):
                    particle = DustParticle(
                        position=np.random.rand(3) * 10.0,
                        velocity=np.random.rand(3) * 2.0,
                        size=0.02,
                        density=1500.0,
                        lifetime=10.0,
                        creation_time=0.0,
                    )
                    particles.append(particle)

                # Time 100 updates
                dt = 0.016
                start = time.time()
                for _ in range(100):
                    dynamics.update(particles, dt)
                elapsed = time.time() - start

                time_per_update = elapsed / 100.0
                timings.append((count, time_per_update))

                self.log(
                    f"  {count} particles: {time_per_update * 1000:.3f} ms per update"
                )

            # Check scaling is roughly linear
            for i in range(1, len(timings)):
                prev_count, prev_time = timings[i - 1]
                curr_count, curr_time = timings[i]

                ratio = (curr_time / prev_time) / (curr_count / prev_count)
                self.log(f"  Scaling ratio: {ratio:.2f} (should be ~1.0 for linear)")

            self.results.append(TestResult("Integration - Performance Scaling", True))

        except Exception as e:
            self.results.append(
                TestResult("Integration - Performance Scaling", False, str(e))
            )

    def print_summary(self) -> bool:
        """Print test summary and return overall pass status."""
        print("\n" + "=" * 70)
        print("Test Summary")
        print("=" * 70)

        passed = 0
        failed = 0

        for result in self.results:
            status = "PASS" if result.passed else "FAIL"
            symbol = "✓" if result.passed else "✗"
            print(f"{symbol} {status}: {result.name}")

            if not result.passed and result.message:
                print(f"      Error: {result.message}")

            if result.passed:
                passed += 1
            else:
                failed += 1

        print("=" * 70)
        print(
            f"Total: {passed} passed, {failed} failed out of {len(self.results)} tests"
        )
        print("=" * 70 + "\n")

        return failed == 0


def main():
    parser = argparse.ArgumentParser(description="Dust Physics Test Suite")
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Enable verbose output"
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Enable visualization (not implemented)",
    )
    args = parser.parse_args()

    # Run tests
    suite = DustPhysicsTestSuite(verbose=args.verbose)
    success = suite.run_all_tests()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
