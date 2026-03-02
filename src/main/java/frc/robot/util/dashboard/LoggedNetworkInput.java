package frc.robot.util.dashboard;

import java.util.ArrayList;
import java.util.List;
import org.jspecify.annotations.NullMarked;

@NullMarked
public abstract class LoggedNetworkInput {
    private static final String PREFIX = "NetworkInputs/";
    private static final List<LoggedNetworkInput> inputs = new ArrayList<>();
    protected final String topicName;

    protected LoggedNetworkInput(String topicName) {
        this.topicName = getAdjustedTopicName(topicName);
        inputs.add(this);
    }

    protected abstract void periodic();

    protected String getAdjustedTopicName(String topicName) {
        return PREFIX + (topicName.startsWith("/") ? topicName.substring(1) : topicName);
    }

    public static void runAllPeriodic() {
        for (final var input : inputs) {
            input.periodic();
        }
    }
}
