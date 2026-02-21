package frc.robot.util.dashboard;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import org.photonvision.targeting.PhotonTrackedTarget;

@NullMarked
public class PhotonTrackedTargetSendable implements Sendable {
    private final @Nullable PhotonTrackedTarget target;

    public PhotonTrackedTargetSendable(@Nullable PhotonTrackedTarget target) {
        this.target = target;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PhotonTrackedTarget");
        builder.addBooleanProperty("Has Target", () -> target != null, null);
        if (target == null) {
            return;
        }
        builder.addIntegerProperty("Target ID", target::getFiducialId, null);
        builder.addDoubleProperty("Target Yaw", target::getYaw, null);
        builder.addDoubleProperty("Target Pitch", target::getPitch, null);
        builder.addDoubleProperty("Target Area", target::getArea, null);
    }
}
