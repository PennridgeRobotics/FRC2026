package frc.robot.util.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import java.util.ArrayList;
import java.util.List;
import org.jspecify.annotations.NullMarked;

@NullMarked
public abstract class LoggedNetworkInput {
    private static final String PREFIX = "Tuning/";
    private static final List<LoggedNetworkInput> inputs = new ArrayList<>();
    protected final String topicName;

    protected LoggedNetworkInput(String topicName) {
        this.topicName = getAdjustedTopicName(topicName);
        inputs.add(this);
    }

    protected abstract void periodic();

    protected static String getAdjustedTopicName(String topicName) {
        if (topicName.startsWith("/")) return topicName.substring(1);
        return PREFIX + topicName;
    }

    public static void runAllPeriodic() {
        for (final var input : inputs) {
            input.periodic();
        }
    }

    public static void publishSendable(String rawTopicName, Sendable sendable) {
        final var topicName = getAdjustedTopicName(rawTopicName);
        final NetworkTable table = NetworkTableInstance.getDefault().getTable(topicName);
        final SendableBuilderImpl builder = new SendableBuilderImpl();
        builder.setTable(table);
        SendableRegistry.publish(sendable, builder);
        builder.startListeners();
        table.getEntry(".name").setString(topicName);
    }
}
