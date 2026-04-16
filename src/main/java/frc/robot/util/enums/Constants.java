package frc.robot.util.enums;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
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
        public static final Distance BUMPERS_HEIGHT = Inches.of(5.0);
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

        public static final boolean VISION_ENABLED = true;

        // Tuning
        // Base standard deviations
        public static final Matrix<N3, N1> PHOTON_SINGLE_TAG_STD_DEVS = VecBuilder.fill(2, 2, 999999);
        public static final Matrix<N3, N1> PHOTON_MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 999999);
        public static final Matrix<N3, N1> LIMELIGHT_SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.8, 0.8, 999999);
        public static final Matrix<N3, N1> LIMELIGHT_MULTI_TAG_STD_DEVS = VecBuilder.fill(0.4, 0.4, 999999);
        // Result std dev = base * (1 + (distance ^ exponent) * multiplier)
        public static final double STD_DEV_DISTANCE_EXPONENT = 3.0;
        public static final double STD_DEV_DISTANCE_MULTIPLIER = 1.0 / 80;

        // Back Camera
        public static final String CAMERA_BACK_NAME = "Arducam_OV9281_Front";
        public static final Translation3d CAMERA_BACK_TRANSLATION = new Translation3d(0.162087, -0.250706, 0.674197);
        public static final Rotation3d CAMERA_BACK_ROTATION = new Rotation3d(-0.007874, 0.014070, -3.123649);
        public static final boolean CAMERA_BACK_USE_IN_POSE_ESTIMATION = true;
        // Front Camera
        public static final String CAMERA_FRONT_NAME = "Arducam_OV9281_Front";
        public static final Translation3d CAMERA_FRONT_TRANSLATION = new Translation3d(-0.045748, -0.249129, 0.580242);
        public static final Rotation3d CAMERA_FRONT_ROTATION = new Rotation3d(0.015923, 0.004286, -0.035962);
        public static final boolean CAMERA_FRONT_USE_IN_POSE_ESTIMATION = false;
    }

    public static final class DriveConstants {
        public static final String SWERVE_CONFIG_DIRECTORY = "swerve"; // + deploy

        public static final LinearAcceleration MAX_LINEAR_ACCELERATION = MetersPerSecondPerSecond.of(15);
        public static final LinearVelocity MAX_LINEAR_SPEED = MetersPerSecond.of(4.2);

        public static final Distance WHEEL_DIAMETER = Inches.of(4);
    }

    public static final class ShootOnTheMoveConstants {
        public static final Distance BALL_DIAMETER = Inches.of(5.906); // need to define this first

        // Measure/tune:
        public static final Distance FLYWHEEL_DIAMETER = Inches.of(3.94);
        public static final Distance EXIT_HEIGHT = Inches.of(20.5); // floor to where ball leaves shooter
        public static final Angle LAUNCH_ANGLE_FROM_HORIZONTAL = Degrees.of(61); // estimated
        public static final double SLIP_FACTOR = 0.851; // 0 = no group, 1 = perfect
        public static final Translation2d LAUNCHER_OFFSET = new Translation2d(Inches.of(7.4), Inches.zero());
        public static final LinearVelocity MAX_VELOCITY_WHILE_SHOOTING = MetersPerSecond.of(1.0);
        public static final Time PHASE_DELAY = Milliseconds.of(100); // vision pipeline latency
        public static final Time MECHANISM_LATENCY = Milliseconds.of(50); // how long the mechanism takes to respond
        public static final Distance HUB_HEIGHT = Inches.of(72) // hub height
                .plus(BALL_DIAMETER.div(2))
                .plus(Inches.of(3)); // buffer

        public static final Mass BALL_MASS = Kilograms.of(0.215);
        public static final double DRAG_COEFFICIENT = 0.11; // 0.47; // smooth sphere
        public static final double MAGNUS_COEFFICIENT = 0.2;
        public static final double AIR_DENSITY = 1.225; // kg/m³
        public static final Time SIM_TIMESTEP = Seconds.of(0.002);
        public static final AngularVelocity RPM_SEARCH_MIN = RPM.of(1500);
        public static final AngularVelocity RPM_SEARCH_MAX = RPM.of(4000); // real limit: 3937
        public static final int ITERATIONS = 25;
        public static final Time MAX_SIM_TIME = Seconds.of(5);
        public static final double MAGNUS_SIGN = 1;

        public static final Angle MAXIMUM_TILT =
                Degrees.of(15); // suppress firing when the chassis tilts past this (bumps/ramps)
        // Heading tolerance tightens as robot speed increases.
        // scaledMaxError = base / (1 + speedScalar * speed). Set to 0 to disable.
        public static final double HEADING_SPEED_SCALAR = 1.0;
        // Heading tolerance scales with distance from hub.
        // Closer = tighter because small angle errors matter more up close.
        // scaledMaxError *= referenceDistance / distance, clamped [0.5, 2.0].
        public static final double HEADING_REFERENCE_DISTANCE = 2.5;
    }

    public static final class PassingConstants {
        public static final LinearVelocity MAX_VELOCITY_WHILE_PASSING = DriveConstants.MAX_LINEAR_SPEED;
        public static final Distance MAXIMUM_DISTANCE_PASSING = Meters.of(7);
        public static final double HEADING_SPEED_SCALAR_PASSING = 0.0;
        public static final double HEADING_REFERENCE_DISTANCE_PASSING = 2.5;

        public static final Translation2d BLUE_HUB_FORWARD_VECTOR = new Translation2d(1, 0);
        public static final Translation2d RED_HUB_FORWARD_VECTOR = new Translation2d(-1, 0);

        public static final Translation2d PASSING_SPOT_LEFT_BLUE =
                new Translation2d(Inches.of(75), FieldConstants.FIELD_WIDTH_Y.minus(Inches.of(75)));
        public static final Translation2d PASSING_SPOT_RIGHT_BLUE = new Translation2d(
                PASSING_SPOT_LEFT_BLUE.getMeasureX(),
                FieldConstants.FIELD_WIDTH_Y.minus(PASSING_SPOT_LEFT_BLUE.getMeasureY()));
        public static final Translation2d PASSING_SPOT_LEFT_RED = new Translation2d(
                FieldConstants.FIELD_LENGTH_X.minus(PASSING_SPOT_RIGHT_BLUE.getMeasureX()),
                PASSING_SPOT_RIGHT_BLUE.getMeasureY());
        public static final Translation2d PASSING_SPOT_RIGHT_RED = new Translation2d(
                FieldConstants.FIELD_LENGTH_X.minus(PASSING_SPOT_LEFT_BLUE.getMeasureX()),
                PASSING_SPOT_LEFT_BLUE.getMeasureY());
    }

    public static final class FuelConstants {
        public static final boolean FUEL_SUBSYSTEM_ENABLED = true;

        public static final int INTAKE_LAUNCHER_LEFT_MOTOR_ID = 10;
        public static final int INTAKE_LAUNCHER_RIGHT_MOTOR_ID = 11;
        public static final int INDEXER_MOTOR_ID = 12;

        public static final Distance FLYWHEEL_RADIUS = ShootOnTheMoveConstants.FLYWHEEL_DIAMETER.div(2);

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
        public static final int INTAKE_LAUNCHER_ENCODER_MEASUREMENT_PERIOD =
                16; // must be between [1, 64]; default = 32ms
        public static final int INTAKE_LAUNCHER_ENCODER_AVERAGE_DEPTH = 2; // must be 1, 2, 4, or 8; default = 8

        public static final AngularVelocity INTAKE_VELOCITY_INTAKE_LAUNCHER = RotationsPerSecond.of(20);
        public static final AngularVelocity INTAKE_VELOCITY_INDEXER = RotationsPerSecond.of(-16);
        public static final AngularVelocity UNJAM_VELOCITY_INTAKE_LAUNCHER = RotationsPerSecond.of(-30);
        public static final AngularVelocity UNJAM_VELOCITY_INDEXER = RotationsPerSecond.of(-18);
        public static final AngularVelocity EJECT_VELOCITY_INTAKE_LAUNCHER = RotationsPerSecond.of(-10);
        public static final AngularVelocity EJECT_VELOCITY_INDEXER = RotationsPerSecond.of(10);
        public static final AngularVelocity LAUNCH_VELOCITY_INDEXER = RotationsPerSecond.of(10);
        public static final AngularVelocity WINDUP_VELOCITY_INDEXER = RotationsPerSecond.of(-3);
        // calculated velocity + LAUNCH_VELOCITY_TOLERANCE = velocity needed to finish winding up
        public static final AngularVelocity LAUNCH_VELOCITY_TOLERANCE = RotationsPerSecond.of(-0.5);
        public static final Voltage MAX_POWER_VOLTAGE = Volts.of(12.0);

        public static final Time SMART_UNJAM_INDEXER_REVERSE_LAUNCH_TIME = Seconds.of(0.42);
        public static final Time SMART_UNJAM_BOTH_REVERSE_PULSE_TIME = Seconds.of(0.20);
        public static final Time SMART_UNJAM_FORCE_LAUNCH_TIME = Seconds.of(0.28);
        public static final Time SMART_UNJAM_EJECT_TIME = Seconds.of(0.35);
        public static final AngularVelocity SMART_UNJAM_MIN_LAUNCHER_SPEED = RotationsPerSecond.of(1);
        public static final AngularVelocity SMART_UNJAM_MIN_INDEXER_SPEED = RotationsPerSecond.of(1);
        public static final AngularVelocity SMART_UNJAM_READY_MIN_LAUNCHER_SPEED = RotationsPerSecond.of(5);
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
        public static final double CLIMB_VALUE = 0.15;
        public static final double CLIMB_FAST_VALUE = 0.5;
        public static final double LOWER_VALUE = -0.3;
        public static final double LOWER_FAST_VALUE = -1.0;

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
        public static final int JOYSTICK_CONTROLLER_PORT = -1;
        public static final boolean USING_JOYSTICK = JOYSTICK_CONTROLLER_PORT >= 0;

        public static final double DRIVE_MIN_INPUT = 0.06; // deadband
        public static final double DRIVE_MAX_INPUT = 0.98;

        public static final double LINEAR_DRIVE_POWER_SCALE = USING_JOYSTICK ? 2 : 3;
        public static final double ROTATE_DRIVE_POWER_SCALE = USING_JOYSTICK ? 3 : 5;
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
