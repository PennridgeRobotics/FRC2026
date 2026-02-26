package frc.robot.util.dashboard;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class Pose3dSendable implements Sendable {
    private final Pose3d pose;

    public Pose3dSendable(Pose3d pose) {
        this.pose = pose;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Pose3d");
        builder.addDoubleProperty("X", pose::getX, null);
        builder.addDoubleProperty("Y", pose::getY, null);
        builder.addDoubleProperty("Z", pose::getZ, null);
        builder.addDoubleProperty("Rotation", () -> pose.getRotation().getZ(), null);
    }
}
