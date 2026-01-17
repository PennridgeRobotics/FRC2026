package org.pennridge.robotics.frc.util.enums;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.units.measure.*;
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

    public static final class PhysicalConstants { // TODO update these numbers
        public static final Distance ROBOT_LENGTH = Inches.of(38);
        public static final Distance ROBOT_WIDTH = Inches.of(32.5);
        public static final Distance WHEEL_CENTERS_DISTANCE_WIDTH = Inches.of(36);
        public static final Distance WHEEL_CENTERS_DISTANCE_LENGTH = Inches.of(30);
        public static final Distance LIMELIGHT_OFFSET_X = Inches.of(10.0); // offset from center
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
    }

    public static final class NavXConstants {
        public static final Distance SENSOR_OFFSET_X = Meters.of(0.1);
        public static final Distance SENSOR_OFFSET_Y = Meters.of(0.1);
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

        public static final String SWERVE_CONFIG_DIRECTORY = "swerve"; // + deploy

        public static final LinearVelocity MAX_LINEAR_SPEED = MetersPerSecond.of(0.5);
        public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(Math.PI / 2);

        public static final int ENCODER_CPR = 1024;
        public static final double GEAR_RATIO = 8.45;
        public static final Time CAN_TIMEOUT = Milliseconds.of(250);
        public static final Current DRIVE_MOTOR_CURRENT_LIMIT = Amps.of(40);
        public static final Distance WHEEL_DIAMETER = Inches.of(6);
        public static final Distance DISTANCE_PER_REV =
                WHEEL_DIAMETER.times(Math.PI).div(GEAR_RATIO);
        public static final Distance TRACK_WIDTH = Inches.of(21.5);
    }

    public static final class FuelConstants {
        public static final int INTAKE_LAUNCHER_MOTOR_ID = 5;
        public static final int FEEDER_MOTOR_ID = 6;

        public static final Voltage VOLTAGE_COMPENSATION = Volts.of(11);

        public static final Current LAUNCHER_MOTOR_CURRENT_LIMIT = Amps.of(40);
        public static final Current FEEDER_MOTOR_CURRENT_LIMIT = Amps.of(40);

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
}
