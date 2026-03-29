package frc.robot.util.dashboard;

import frc.robot.util.Stopwatch;
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
        final var stopwatch = new Stopwatch("LoggedNetworkInput.runAllPeriodic()");
        for (final var input : inputs) {
            input.periodic();
            stopwatch.logTime(input.topicName);
        }
    }
}
