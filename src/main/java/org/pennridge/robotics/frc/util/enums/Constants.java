package org.pennridge.robotics.frc.util.enums;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import org.jspecify.annotations.NullMarked;

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

    public static final class PathPlannerConstants {
        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                MetersPerSecond.of(2),
                MetersPerSecondPerSecond.of(1.75),
                RadiansPerSecond.of(0.5 * Math.PI),
                RadiansPerSecondPerSecond.of(1 * Math.PI));
    }

    public static final class VisionConstants {
        public static final String LIMELIGHT_NAME = "limelight";

        public static final boolean VISION_ENABLED = false;
    }

    public static final class DriveConstants {
        public static final String SWERVE_CONFIG_DIRECTORY = "swerve"; // + deploy

        public static final LinearVelocity MAX_LINEAR_SPEED = MetersPerSecond.of(0.5);

        public static final Current DRIVE_MOTOR_CURRENT_LIMIT = Amps.of(40);
        public static final Distance WHEEL_DIAMETER = Inches.of(4);
    }

    public static final class FuelConstants {
        public static final int INTAKE_LAUNCHER_MOTOR_ID = 14;
        public static final int FEEDER_MOTOR_ID = 15;

        public static final Current MOTOR_CURRENT_LIMIT = Amps.of(40);

        public static final Voltage INTAKE_FEEDER_VOLTAGE = Volts.of(-12);
        public static final Voltage INTAKE_INTAKE_VOLTAGE = Volts.of(10);
        public static final Voltage LAUNCHING_FEEDER_VOLTAGE = Volts.of(9);
        public static final Voltage LAUNCHING_LAUNCHER_VOLTAGE = Volts.of(10.6);
        public static final Voltage SPIN_UP_FEEDER_VOLTAGE = Volts.of(-6);
        public static final Time SPIN_UP_SECONDS = Seconds.of(1);
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
}
