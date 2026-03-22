package frc.robot.util.dashboard;

import com.ctre.phoenix6.hardware.core.CorePigeon2;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class Pigeon2Sendable implements Sendable {
    private final CorePigeon2 pigeon;

    public Pigeon2Sendable(CorePigeon2 pigeon) {
        this.pigeon = pigeon;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Yaw (Z)", () -> pigeon.getYaw(false).getValueAsDouble(), pigeon::setYaw);
        builder.addDoubleProperty("Pitch (Y)", () -> pigeon.getPitch(false).getValueAsDouble(), null);
        builder.addDoubleProperty("Roll (X)", () -> pigeon.getRoll(false).getValueAsDouble(), null);
        builder.addDoubleProperty(
                "Accumulated X", () -> pigeon.getAccumGyroX(false).getValueAsDouble(), null);
        builder.addDoubleProperty(
                "Accumulated Y", () -> pigeon.getAccumGyroY(false).getValueAsDouble(), null);
        builder.addDoubleProperty(
                "Accumulated Z", () -> pigeon.getAccumGyroZ(false).getValueAsDouble(), null);
        builder.addDoubleProperty(
                "Angular Velocity X",
                () -> pigeon.getAngularVelocityXWorld(false).getValueAsDouble(),
                null);
        builder.addDoubleProperty(
                "Angular Velocity Y",
                () -> pigeon.getAngularVelocityYWorld(false).getValueAsDouble(),
                null);
        builder.addDoubleProperty(
                "Angular Velocity Z",
                () -> pigeon.getAngularVelocityZWorld(false).getValueAsDouble(),
                null);
        builder.addDoubleProperty(
                "Acceleration X", () -> pigeon.getAccelerationX(false).getValueAsDouble(), null);
        builder.addDoubleProperty(
                "Acceleration Y", () -> pigeon.getAccelerationY(false).getValueAsDouble(), null);
        builder.addDoubleProperty(
                "Acceleration Z", () -> pigeon.getAccelerationZ(false).getValueAsDouble(), null);
    }
}
