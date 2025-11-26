package org.pennridge.robotics.frc.util.enums;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import org.jetbrains.annotations.NotNull;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to
 * reduce verbosity.
 */
public final class Constants {

    public static final class PhysicalConstants {
        public static final @NotNull Distance ROBOT_LENGTH = Inches.of(38);
        public static final @NotNull Distance ROBOT_WIDTH = Inches.of(32.5);
        public static final @NotNull Distance LIMELIGHT_OFFSET_X = Inches.of(10.0); // offset from center
    }

    public static final class PathPlannerConstants {
        public static final @NotNull PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                MetersPerSecond.of(2),
                MetersPerSecondPerSecond.of(1.75),
                RadiansPerSecond.of(0.5 * Math.PI),
                RadiansPerSecondPerSecond.of(1 * Math.PI));
    }

    public static final class VisionConstants {
        public static final @NotNull String LIMELIGHT_NAME = "limelight";
    }

    public static final class NavXConstants {
        public static final @NotNull Distance SENSOR_OFFSET_X = Meters.of(0.1);
        public static final @NotNull Distance SENSOR_OFFSET_Y = Meters.of(0.1);
    }

    public static final class DriveConstants {
        public static final int LEFT_LEADER_ID = 1;
        public static final int LEFT_FOLLOWER_ID = 2;
        public static final int RIGHT_LEADER_ID = 3;
        public static final int RIGHT_FOLLOWER_ID = 4;

        public static final int[] LEFT_ENCODER_PORTS = {1, 2};
        public static final int[] RIGHT_ENCODER_PORTS = {3, 4};
        public static final boolean LEFT_ENCODER_REVERSED = false;
        public static final boolean RIGHT_ENCODER_REVERSED = true;

        public static final int ENCODER_CPR = 1024;
        public static final double GEAR_RATIO = 8.45;
        public static final @NotNull Time CAN_TIMEOUT = Milliseconds.of(250);
        public static final @NotNull Current DRIVE_MOTOR_CURRENT_LIMIT = Amps.of(40);
        public static final @NotNull Distance WHEEL_DIAMETER = Inches.of(6);
        public static final @NotNull Distance DISTANCE_PER_REV =
                WHEEL_DIAMETER.times(Math.PI).div(GEAR_RATIO);
        public static final @NotNull Distance TRACK_WIDTH = Inches.of(21.5);
    }

    public static final class ControllerConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }
}
