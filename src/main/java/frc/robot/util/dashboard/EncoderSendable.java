package frc.robot.util.dashboard;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class EncoderSendable implements Sendable {
    private final RelativeEncoder encoder;

    public EncoderSendable(final RelativeEncoder encoder) {
        this.encoder = encoder;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Encoder");
        builder.addDoubleProperty("Distance", encoder::getPosition, encoder::setPosition);
        builder.addDoubleProperty("Speed", encoder::getVelocity, null);
    }
}
