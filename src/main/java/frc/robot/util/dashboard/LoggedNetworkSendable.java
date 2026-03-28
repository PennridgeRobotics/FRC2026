package frc.robot.util.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class LoggedNetworkSendable<T extends Sendable> extends LoggedNetworkInput {
    private final T sendable;

    public LoggedNetworkSendable(String rawTopicName, T sendable) {
        super(rawTopicName);
        this.sendable = sendable;
        final NetworkTable table = NetworkTableInstance.getDefault().getTable(topicName);
        final SendableBuilderImpl builder = new SendableBuilderImpl();
        builder.setTable(table);
        SendableRegistry.publish(sendable, builder);
        builder.startListeners();
        table.getEntry(".name").setString(topicName);
    }

    @Override
    protected void periodic() {
        try {
            SendableRegistry.update(sendable);
        } catch (Exception e) {
            DriverStation.reportError("Error updating LoggedNetworkSendable (" + topicName + ")", e.getStackTrace());
        }
    }
}
