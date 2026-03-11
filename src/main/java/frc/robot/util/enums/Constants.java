package frc.robot.util.enums;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;
import org.jspecify.annotations.NullMarked;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to
 * reduce verbosity.
 */
@NullMarked
public final class Constants {

    public static final class PhysicalConstants { // TODO update length/width depending on bumper size
        public static final Distance BUMPERS_WIDTH = Inches.of(3.375);
        public static final Distance WHEEL_CENTERS_DISTANCE_LENGTH_X = Inches.of(18.5); // 0.4699m
        public static final Distance WHEEL_CENTERS_DISTANCE_WIDTH_Y = Inches.of(23.5); // 0.5969m
        public static final Distance ROBOT_FULL_LENGTH_X =
                WHEEL_CENTERS_DISTANCE_LENGTH_X.plus(BUMPERS_WIDTH.times(2)); // 25.25in/0.6414m
        public static final Distance ROBOT_FULL_WIDTH_Y =
                WHEEL_CENTERS_DISTANCE_WIDTH_Y.plus(BUMPERS_WIDTH.times(2)); // 30.25in/0.7684m
        public static final Distance ROBOT_LENGTH_X = ROBOT_FULL_LENGTH_X;
        public static final Distance ROBOT_WIDTH_Y = ROBOT_FULL_WIDTH_Y;
        public static final Distance ROBOT_TRENCH_BACK_OFFSET = Inches.of(4.88);
        public static final Distance ROBOT_TRENCH_FRONT_OFFSET = Inches.of(0.875);
        public static final Distance ROBOT_WIDTH_Y_TRENCH = ROBOT_WIDTH_Y.minus(Inches.of(4.6 * 2));
    }

    public static final class BLineConstants {}

    public static final class VisionConstants {
        public static final String LIMELIGHT_NAME = "limelight";

        public static final boolean VISION_ENABLED = false;

        // Tuning
        // Base standard deviations
        public static final Matrix<N3, N1> PHOTON_SINGLE_TAG_STD_DEVS = VecBuilder.fill(2, 2, 999999);
        public static final Matrix<N3, N1> PHOTON_MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 999999);
        public static final Matrix<N3, N1> LIMELIGHT_SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.8, 0.8, 999999);
        public static final Matrix<N3, N1> LIMELIGHT_MULTI_TAG_STD_DEVS = VecBuilder.fill(0.4, 0.4, 999999);
        // Result std dev = base * (1 + (distance ^ exponent) * multiplier)
        public static final double STD_DEV_DISTANCE_EXPONENT = 2.0;
        public static final double STD_DEV_DISTANCE_MULTIPLIER = 1.0 / 30;

        // Camera 1
        public static final String CAMERA_1_NAME = "Arducam_OV9281_1";
        public static final Translation3d CAMERA_1_TRANSLATION =
                new Translation3d(Inches.of(6.26), Inches.of(-15.74), Inches.of(14.32)); // Robot to cam
        public static final Rotation3d CAMERA_1_ROTATION =
                new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)); // Robot to cam
    }

    public static final class DriveConstants {
        public static final String SWERVE_CONFIG_DIRECTORY = "swerve"; // + deploy

        public static final LinearAcceleration MAX_LINEAR_ACCELERATION = MetersPerSecondPerSecond.of(10);
        public static final LinearVelocity MAX_LINEAR_SPEED = MetersPerSecond.of(4.2);

        public static final Distance WHEEL_DIAMETER = Inches.of(4);
    }

    public static final class FuelConstants {
        public static final boolean FUEL_SUBSYSTEM_ENABLED = true;

        public static final int INTAKE_LAUNCHER_LEFT_MOTOR_ID = 10;
        public static final int INTAKE_LAUNCHER_RIGHT_MOTOR_ID = 11;
        public static final int INDEXER_MOTOR_ID = 12;

        public static final Distance WHEEL_RADIUS = Inches.of(2);

        public static final boolean INTAKE_LAUNCHER_INVERTED = false;
        public static final boolean INDEXER_INVERTED = false;
        public static final Voltage INTAKE_LAUNCHER_VOLTAGE_COMP = Volts.of(12);
        public static final Voltage INDEXER_VOLTAGE_COMP = Volts.of(12);
        public static final Current INTAKE_LAUNCHER_CURRENT_LIMIT = Amps.of(60);
        public static final Current INDEXER_CURRENT_LIMIT = Amps.of(60);
        public static final Time INTAKE_LAUNCHER_RAMP_RATE = Seconds.of(0.2);
        public static final Time INDEXER_RAMP_RATE = Seconds.of(0.2);
        public static final Time WINDUP_TIMEOUT = Seconds.of(4.0);
        public static final Time UNJAM_AFTER_LAUNCH_TIME = Seconds.of(0);
        public static final MotorMode INTAKE_LAUNCHER_MOTOR_MODE = MotorMode.BRAKE;
        public static final MotorMode INDEXER_MOTOR_MODE = MotorMode.BRAKE;
        public static final MechanismGearing INTAKE_LAUNCHER_GEARING = new MechanismGearing(60.0 / 40);
        public static final MechanismGearing INDEXER_GEARING = new MechanismGearing(32.0 / 18);

        public static final AngularVelocity INTAKE_VELOCITY_INTAKE_LAUNCHER = RotationsPerSecond.of(20);
        public static final AngularVelocity INTAKE_VELOCITY_INDEXER = RotationsPerSecond.of(-16);
        public static final AngularVelocity UNJAM_VELOCITY_INTAKE_LAUNCHER = RotationsPerSecond.of(-10);
        public static final AngularVelocity UNJAM_VELOCITY_INDEXER = RotationsPerSecond.of(-10);
        public static final AngularVelocity EJECT_VELOCITY_INTAKE_LAUNCHER = RotationsPerSecond.of(-10);
        public static final AngularVelocity EJECT_VELOCITY_INDEXER = RotationsPerSecond.of(10);
        public static final AngularVelocity LAUNCH_VELOCITY_INDEXER = RotationsPerSecond.of(10);
        public static final AngularVelocity WINDUP_VELOCITY_INDEXER = RotationsPerSecond.of(-3);
        // calculated velocity + LAUNCH_VELOCITY_TOLERANCE = velocity needed to finish winding up
        public static final AngularVelocity LAUNCH_VELOCITY_TOLERANCE = RotationsPerSecond.of(-0.5);
        public static final Voltage MAX_POWER_VOLTAGE = Volts.of(12.0);
    }

    public static final class ClimberConstants {
        public static boolean CLIMBER_ENABLED = true;

        public static final int CLIMBER_MOTOR_ID = 9;
        public static final boolean CLIMBER_INVERTED = true;
        public static final MotorMode IDLE_MODE = MotorMode.BRAKE;
        public static final MechanismGearing CLIMBER_GEARING = new MechanismGearing(80.0, 60.0 / 20, 28.0 / 10);
        public static final Current CURRENT_LIMIT = Amps.of(40);
        public static final Current STALL_CURRENT = Amps.of(10);
        public static final Time RAMP_RATE = Seconds.of(0);
        public static final Voltage VOLTAGE_COMPENSATION = Volts.of(12.0);

        // open loop
        public static final double CLIMB_VALUE = 0.2;
        public static final double CLIMB_FAST_VALUE = 0.6;
        public static final double LOWER_VALUE = -0.2;
        public static final double LOWER_FAST_VALUE = -0.6;

        public static final Angle MINIMUM_ANGLE = Degrees.of(-53.5);
        public static final Angle MAXIMUM_ANGLE = Degrees.of(110.0);
        public static final Angle HORIZONTAL_ANGLE = Degrees.of(0);
        public static final Angle VERTICAL_ANGLE = Degrees.of(90);
        public static final Angle ARMED_ANGLE = Degrees.of(-16);
        public static final Angle CLIMBED_ANGLE = Degrees.of(70);

        public static final Angle TOLERANCE_ANGLE = Degrees.of(1);
    }

    public static final class ControllerConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static final boolean OPERATOR_ENABLED = true;

        public static final double DRIVE_MIN_INPUT = 0.01; // deadband
        public static final double DRIVE_MAX_INPUT = 0.98;
    }

    public static class FieldConstants {
        public static final AprilTagFieldLayout APRIL_TAGS =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        public static final Distance FIELD_LENGTH_X = Inches.of(651.22);
        public static final Distance FIELD_WIDTH_Y = Inches.of(317.69);

        public static final Distance ALLIANCE_ZONE = Inches.of(156.61);

        public static final Translation2d HUB_BLUE = new Translation2d(Inches.of(182.11), FIELD_WIDTH_Y.div(2));
        public static final Translation2d HUB_RED =
                new Translation2d(FIELD_LENGTH_X.minus(Inches.of(182.11)), FIELD_WIDTH_Y.div(2));

        public static final Distance HUB_LENGTH_Y = Inches.of(47);
        public static final Distance HUB_WIDTH_X = Inches.of(47);

        public static final Distance TRENCH_X = Inches.of(182.11 - (3.5 / 2.0)); // account for trench bar width
        public static final Distance TRENCH_TO_EDGE_Y = Inches.of(50.35);

        private static final Distance BUMP_X = Inches.of(182.11);
        private static final Distance BUMP_TO_EDGE_Y = TRENCH_TO_EDGE_Y.plus(Inches.of(12));
        private static final Distance BUMP_LENGTH = Inches.of(73);

        // 3.5 feet away from robot width
        private static final Distance BUMP_EXTENSION_X =
                PhysicalConstants.ROBOT_WIDTH_Y.div(2).plus(Inches.of(42));
        // bump zones are only when the full robot (while diagonal) would fit on the bump
        private static final Distance BUMP_CLEARANCE_Y = Inches.of(
                Math.hypot(PhysicalConstants.ROBOT_WIDTH_Y.in(Inches), PhysicalConstants.ROBOT_LENGTH_X.in(Inches))
                        / 2.0);

        public static final Rectangle2d[] BUMP_ZONES = {
            new Rectangle2d(
                    new Translation2d(BUMP_X.minus(BUMP_EXTENSION_X), BUMP_TO_EDGE_Y.plus(BUMP_CLEARANCE_Y)),
                    new Translation2d(
                            BUMP_X.plus(BUMP_EXTENSION_X),
                            BUMP_TO_EDGE_Y.plus(BUMP_LENGTH).minus(BUMP_CLEARANCE_Y))),
            new Rectangle2d(
                    new Translation2d(
                            BUMP_X.minus(BUMP_EXTENSION_X),
                            FIELD_WIDTH_Y.minus(BUMP_TO_EDGE_Y.plus(BUMP_LENGTH).minus(BUMP_CLEARANCE_Y))),
                    new Translation2d(
                            BUMP_X.plus(BUMP_EXTENSION_X), FIELD_WIDTH_Y.minus(BUMP_TO_EDGE_Y.plus(BUMP_CLEARANCE_Y)))),
            new Rectangle2d(
                    new Translation2d(
                            FIELD_LENGTH_X.minus(BUMP_X.plus(BUMP_EXTENSION_X)),
                            FIELD_WIDTH_Y.minus(BUMP_TO_EDGE_Y.plus(BUMP_LENGTH).minus(BUMP_CLEARANCE_Y))),
                    new Translation2d(
                            FIELD_LENGTH_X.minus(BUMP_X.minus(BUMP_EXTENSION_X)),
                            FIELD_WIDTH_Y.minus(BUMP_TO_EDGE_Y.plus(BUMP_CLEARANCE_Y)))),
            new Rectangle2d(
                    new Translation2d(
                            FIELD_LENGTH_X.minus(BUMP_X.plus(BUMP_EXTENSION_X)), BUMP_TO_EDGE_Y.plus(BUMP_CLEARANCE_Y)),
                    new Translation2d(
                            FIELD_LENGTH_X.minus(BUMP_X.minus(BUMP_EXTENSION_X)),
                            BUMP_TO_EDGE_Y.plus(BUMP_LENGTH).minus(BUMP_CLEARANCE_Y)))
        };
    }

    public static class LightConstants {
        public static final boolean LIGHTS_ENABLED = false;
        public static final int CANDLE_ID = 16;
    }

    public static class MiscConstants {
        public static final int POWER_DISTRIBUTION_HUB_ID = 14;
    }
}
