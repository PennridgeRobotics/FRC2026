package frc.robot.util.dashboard;

import com.studica.frc.AHRS;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class AHRSSendable implements Sendable {
    private final AHRS ahrs;

    public AHRSSendable(AHRS ahrs) {
        this.ahrs = ahrs;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
                "Speed", () -> Math.sqrt(Math.pow(ahrs.getVelocityX(), 2) + Math.pow(ahrs.getVelocityY(), 2)), null);
        builder.addDoubleProperty("Angle", ahrs::getAngle, null);
        builder.addDoubleProperty("Pitch", ahrs::getPitch, null);
        builder.addDoubleProperty("Roll", ahrs::getRoll, null);
        builder.addDoubleProperty(
                "Displacement",
                () -> Math.sqrt(Math.pow(ahrs.getDisplacementX(), 2) + Math.pow(ahrs.getDisplacementY(), 2)),
                null);
        builder.addBooleanProperty("NavX connected", ahrs::isConnected, null);
    }
}
