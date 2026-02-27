package frc.robot.util.dashboard;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.lib.Elastic;

public class CANBusLoadSendable implements Sendable {
    int elapsed = 0;
    double utilPercent = RobotController.getCANStatus().percentBusUtilization;
    double preUtilPrecent = 0.0;
    double utilPercents = 0.0;
    java.util.List<Double> utilPercentsList = new java.util.ArrayList<>();
    int red = 0;
    int green = 255;

    @Override
    public void initSendable(SendableBuilder builder) {
        Notify("buffer");

        // Placeholder properties for feeder and intake positions, velocities, and voltages

        /*
        builder.addDoubleProperty("Feeder Position", () -> 0.0, null);
        builder.addDoubleProperty("Intake Position", () -> 0.0, null);

        builder.addDoubleProperty("Feeder Velocity", () -> 0.0, null);
        builder.addDoubleProperty("Intake Velocity", () -> 0.0, null);

        builder.addDoubleProperty("Intaking Feeder Voltage", () -> 0.0, null);
        builder.addDoubleProperty("Intaking Intake Voltage", () -> 0.0, null);
        builder.addDoubleProperty("Launching Feeder Voltage", () -> 0.0, null);
        builder.addDoubleProperty("Launching Launcher Voltage", () -> 0.0, null);
        builder.addDoubleProperty("Spin Up Feeder Voltage", () -> 0.0, null);
        */

        // CAN Bus usage properties

        // Current usage percentage
        builder.addDoubleProperty(
                "CAN Bus Percentage", () -> RobotController.getCANStatus().percentBusUtilization, null);

        // Average usage percentage over the last few seconds
        builder.addDoubleProperty(
                "CAN Bus Average %",
                () -> {
                    utilPercent = RobotController.getCANStatus().percentBusUtilization;
                    utilPercentsList.add(utilPercent);
                    elapsed++;
                    if (elapsed > 240) {
                        utilPercentsList.remove(0);
                    }
                    double sum = 0.0;
                    for (double d : utilPercentsList) {
                        sum += d;
                    }
                    return sum / utilPercentsList.size();
                },
                null);

        // Full average usage percentage since the start
        builder.addDoubleProperty(
                "CAN Bus Full Average %",
                () -> {
                    utilPercent = RobotController.getCANStatus().percentBusUtilization;
                    utilPercents += utilPercent;
                    elapsed++;
                    return utilPercents / elapsed;
                },
                null);

        // A color that changes from green to yellow to red based on the current usage percentage
        builder.addStringProperty(
                // 0%-60% FF0000 -> 60%-100% 00FF00 gradient based on percentage
                "CAN Bus Status", () -> updateColor(), null);

        Notify("init");
    }

    public String updateColor() {
        preUtilPrecent = utilPercent;
        utilPercent = RobotController.getCANStatus().percentBusUtilization;

        // Notifications
        if (utilPercent > 0.7 && preUtilPrecent >= 0.7) {
            Notify("highBusPercentWarning");
        }
        if (utilPercent > 0.8 && preUtilPrecent >= 0.8) {
            Notify("highBusPercentCritical");
        }

        if (utilPercent > 0.4) {
            red = (int) (255 * (utilPercent - 0.4) * 5); // Scale from 0 to 255 as it goes from 40% to 60%
            if (red > 255) red = 255; // Cap at 255
            if (utilPercent > 0.6) {
                green = (int)
                        (255 - (255 * (utilPercent - 0.6) * 2.5)); // Scale from 255 to 0 as it goes from 60% to 100%
            }
        } else {
            red = 0;
            green = 255;
        }
        Color color = new Color(red, green, 0);
        return color.toHexString();
    }

    public void Notify(String message) {
        // Buffer to make sure notifications are shown due to bug of first notification not being shown.
        if (message.equals("buffer")) {
            Elastic.Notification buffer =
                    new Elastic.Notification(Elastic.Notification.NotificationLevel.INFO, "", "", 2147483647);
            Elastic.sendNotification(buffer);
        } else if (message.equals("init")) {
            Elastic.Notification init = new Elastic.Notification(
                    Elastic.Notification.NotificationLevel.INFO, "Welcome", "Initialization Sucessful :)", 10000);
            Elastic.sendNotification(init);
        } else if (message.equals("highBusPercentWarning")) {
            Elastic.Notification highBusPercentWarning = new Elastic.Notification(
                    Elastic.Notification.NotificationLevel.WARNING,
                    "High Usage",
                    "The CAN Bus usage is above 70%",
                    10000);
            Elastic.sendNotification(highBusPercentWarning);
        } else if (message.equals("highBusPercentCritical")) {
            Elastic.Notification highBusPercentCritical = new Elastic.Notification(
                    Elastic.Notification.NotificationLevel.ERROR,
                    "Critical Usage",
                    "The CAN Bus usage is above 80%",
                    10000);
            Elastic.sendNotification(highBusPercentCritical);
        }
    }
}
