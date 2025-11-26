package org.pennridge.robotics.frc.util.dashboard;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.jetbrains.annotations.NotNull;

public class EncoderSendable implements Sendable {
    private final @NotNull RelativeEncoder encoder;

    public EncoderSendable(final @NotNull RelativeEncoder encoder) {
        this.encoder = encoder;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Encoder");
        builder.addDoubleProperty("Distance", encoder::getPosition, encoder::setPosition);
        builder.addDoubleProperty("Speed", encoder::getVelocity, null);
    }
}
