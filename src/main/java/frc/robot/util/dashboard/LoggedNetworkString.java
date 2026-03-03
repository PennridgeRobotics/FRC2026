package frc.robot.util.dashboard;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class LoggedNetworkString extends LoggedNetworkInput implements Supplier<String>, Consumer<String> {
    private final StringEntry entry;
    private String currentValue;

    public LoggedNetworkString(String rawTopicName, String defaultValue) {
        super(rawTopicName);
        entry = NetworkTableInstance.getDefault().getStringTopic(topicName).getEntry(defaultValue);
        entry.set(defaultValue);
        currentValue = entry.get();
    }

    public void set(String value) {
        entry.set(value);
    }

    @Override
    protected void periodic() {
        currentValue = entry.get();
    }

    @Override
    public String get() {
        return currentValue;
    }

    @Override
    public void accept(String value) {
        entry.set(value);
    }
}
