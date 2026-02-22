package frc.robot.util.enums;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.BLine.Path;
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
        public static final Distance ROBOT_LENGTH = Inches.of(26.5);
        public static final Distance ROBOT_WIDTH = Inches.of(31.5);
        public static final Distance WHEEL_CENTERS_DISTANCE_LENGTH = Inches.of(18.5);
        public static final Distance WHEEL_CENTERS_DISTANCE_WIDTH = Inches.of(23.5);
    }

    public static final class BLineConstants {
        public static final LinearVelocity MAX_LINEAR_VELOCITY = MetersPerSecond.of(2); // 4.5
        public static final LinearAcceleration MAX_LINEAR_ACCELERATION = MetersPerSecondPerSecond.of(1.75); // 12.0
        public static final AngularVelocity MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(180); // 540
        public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = DegreesPerSecondPerSecond.of(360); // 860
        public static final Distance END_TRANSLATION_TOLERANCE = Meters.of(0.03);
        public static final Angle END_ROTATION_TOLERANCE = Degrees.of(2);
        public static final Distance INTERMEDIATE_HANDOFF_RADIUS = Meters.of(0.2);

        public static final Path.DefaultGlobalConstraints GLOBAL_CONSTRAINTS = new Path.DefaultGlobalConstraints(
                MAX_LINEAR_VELOCITY.in(MetersPerSecond),
                MAX_LINEAR_ACCELERATION.in(MetersPerSecondPerSecond),
                MAX_ANGULAR_VELOCITY.in(DegreesPerSecond),
                MAX_ANGULAR_ACCELERATION.in(DegreesPerSecondPerSecond),
                END_TRANSLATION_TOLERANCE.in(Meters),
                END_ROTATION_TOLERANCE.in(Degrees),
                INTERMEDIATE_HANDOFF_RADIUS.in(Meters));
    }

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
                new Translation3d(Inches.of(0), Inches.of(0), Inches.of(0)); // Robot to cam
        public static final Rotation3d CAMERA_1_ROTATION =
                new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)); // Robot to cam
    }

    public static final class DriveConstants {
        public static final String SWERVE_CONFIG_DIRECTORY = "swerve"; // + deploy

        public static final LinearAcceleration MAX_LINEAR_ACCELERATION = MetersPerSecondPerSecond.of(20);
        public static final LinearVelocity MAX_LINEAR_SPEED = MetersPerSecond.of(1.0);

        public static final Current DRIVE_MOTOR_CURRENT_LIMIT = Amps.of(40);
        public static final Distance WHEEL_DIAMETER = Inches.of(4);
    }

    public static final class FuelConstants {
        public static final boolean FUEL_SUBSYSTEM_ENABLED = false;

        public static final int INTAKE_LAUNCHER_MOTOR_ID = 14;
        public static final int FEEDER_MOTOR_ID = 15;

        public static final Current MOTOR_CURRENT_LIMIT = Amps.of(40);

        public static final double INDEXER_INTAKING_PERCENT = -.8;
        public static final double INDEXER_LAUNCHING_PERCENT = 0.6;
        public static final double INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT = -0.5;

        public static final double INTAKE_INTAKING_PERCENT = 0.6;
        public static final double LAUNCHING_LAUNCHER_PERCENT = .85;
        public static final double INTAKE_EJECT_PERCENT = -0.8;

        public static final Voltage INTAKE_FEEDER_VOLTAGE = Volts.of(-12);
        public static final Voltage INTAKE_INTAKE_VOLTAGE = Volts.of(10);
        public static final Voltage LAUNCHING_FEEDER_VOLTAGE = Volts.of(9);
        public static final Voltage LAUNCHING_LAUNCHER_VOLTAGE = Volts.of(10.6);
        public static final Voltage SPIN_UP_FEEDER_VOLTAGE = Volts.of(-6);
        public static final Time SPIN_UP_SECONDS = Seconds.of(1);
    }

    public static final class ClimberConstants {
        public static boolean CLIMBER_ENABLED = false;

        public static final int CLIMBER_MOTOR_ID = 9;
        public static final MotorMode IDLE_MODE = MotorMode.BRAKE;
        public static final MechanismGearing CLIMBER_GEARING = new MechanismGearing(100.0, 60.0 / 20, 28.0 / 10);
        public static final Current CURRENT_LIMIT = Amps.of(40);
        public static final Current STALL_CURRENT = Amps.of(10);
        public static final Time RAMP_RATE = Seconds.of(0.25);

        public static final Angle MINIMUM_ANGLE = Degrees.of(-53.5);
        public static final Angle HORIZONTAL_ANGLE = Degrees.of(0);
        public static final Angle CLIMBED_ANGLE = Degrees.of(70);
    }

    public static final class ControllerConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static final double DRIVE_MIN_INPUT = 0.01; // deadband
        public static final double DRIVE_MAX_INPUT = 0.98;
    }

    public static class FieldConstants {
        public static final AprilTagFieldLayout APRIL_TAGS =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        public static final Distance FIELD_LENGTH = Inches.of(651.22);
        public static final Distance FIELD_WIDTH = Inches.of(317.69);

        public static final Distance ALLIANCE_ZONE = Inches.of(156.61);

        public static final Translation2d HUB_BLUE = new Translation2d(Inches.of(182.11), FIELD_WIDTH.div(2));
        public static final Translation2d HUB_RED =
                new Translation2d(FIELD_LENGTH.minus(Inches.of(182.11)), FIELD_WIDTH.div(2));

        private static final Distance BUMP_X = Inches.of(182.11);
        private static final Distance BUMP_TO_EDGE_Y = Inches.of(50.35 + 12);
        private static final Distance BUMP_LENGTH = Inches.of(73);

        // 3.5 feet away from robot width
        private static final Distance BUMP_EXTENSION_X =
                PhysicalConstants.ROBOT_WIDTH.div(2).plus(Inches.of(42));
        // bump zones are only when the full robot (while diagonal) would fit on the bump
        private static final Distance BUMP_CLEARANCE_Y = Inches.of(
                Math.hypot(PhysicalConstants.ROBOT_WIDTH.in(Inches), PhysicalConstants.ROBOT_LENGTH.in(Inches)) / 2.0);

        public static final Rectangle2d[] BUMP_ZONES = {
            new Rectangle2d(
                    new Translation2d(BUMP_X.minus(BUMP_EXTENSION_X), BUMP_TO_EDGE_Y.plus(BUMP_CLEARANCE_Y)),
                    new Translation2d(
                            BUMP_X.plus(BUMP_EXTENSION_X),
                            BUMP_TO_EDGE_Y.plus(BUMP_LENGTH).minus(BUMP_CLEARANCE_Y))),
            new Rectangle2d(
                    new Translation2d(
                            BUMP_X.minus(BUMP_EXTENSION_X),
                            FIELD_WIDTH.minus(BUMP_TO_EDGE_Y.plus(BUMP_LENGTH).minus(BUMP_CLEARANCE_Y))),
                    new Translation2d(
                            BUMP_X.plus(BUMP_EXTENSION_X), FIELD_WIDTH.minus(BUMP_TO_EDGE_Y.plus(BUMP_CLEARANCE_Y)))),
            new Rectangle2d(
                    new Translation2d(
                            FIELD_LENGTH.minus(BUMP_X.plus(BUMP_EXTENSION_X)),
                            FIELD_WIDTH.minus(BUMP_TO_EDGE_Y.plus(BUMP_LENGTH).minus(BUMP_CLEARANCE_Y))),
                    new Translation2d(
                            FIELD_LENGTH.minus(BUMP_X.minus(BUMP_EXTENSION_X)),
                            FIELD_WIDTH.minus(BUMP_TO_EDGE_Y.plus(BUMP_CLEARANCE_Y)))),
            new Rectangle2d(
                    new Translation2d(
                            FIELD_LENGTH.minus(BUMP_X.plus(BUMP_EXTENSION_X)), BUMP_TO_EDGE_Y.plus(BUMP_CLEARANCE_Y)),
                    new Translation2d(
                            FIELD_LENGTH.minus(BUMP_X.minus(BUMP_EXTENSION_X)),
                            BUMP_TO_EDGE_Y.plus(BUMP_LENGTH).minus(BUMP_CLEARANCE_Y)))
        };
    }

    public static class LightConstants {
        public static final boolean LIGHTS_ENABLED = false;
        public static final int CANDLE_ID = 16;
    }
}
