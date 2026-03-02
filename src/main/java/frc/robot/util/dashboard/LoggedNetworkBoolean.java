package frc.robot.util.dashboard;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.function.BooleanConsumer;
import java.util.function.BooleanSupplier;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class LoggedNetworkBoolean extends LoggedNetworkInput implements BooleanSupplier, BooleanConsumer {
    private final BooleanEntry entry;
    private boolean currentValue;

    public LoggedNetworkBoolean(String rawTopicName, boolean defaultValue) {
        super(rawTopicName);
        entry = NetworkTableInstance.getDefault().getBooleanTopic(topicName).getEntry(defaultValue);
    }

    public void set(boolean value) {
        entry.set(value);
    }

    @Override
    protected void periodic() {
        currentValue = entry.get();
    }

    @Override
    public boolean getAsBoolean() {
        return currentValue;
    }

    @Override
    public void accept(boolean value) {
        entry.set(value);
    }
}
