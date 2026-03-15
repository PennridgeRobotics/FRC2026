package frc.robot.util.dashboard;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class LoggedNetworkString extends LoggedNetworkInput implements Supplier<String>, Consumer<String> {
    private final List<Consumer<String>> listeners = new ArrayList<>();
    private final StringEntry entry;
    private String currentValue;

    public LoggedNetworkString(String rawTopicName, String defaultValue) {
        super(rawTopicName);
        entry = NetworkTableInstance.getDefault().getStringTopic(topicName).getEntry(defaultValue);
        entry.set(defaultValue);
        currentValue = entry.get();
    }

    public void set(String value) {
        set(value, true);
    }

    public void set(String value, boolean triggerListeners) {
        if (currentValue.equals(value)) return;
        entry.set(value);
        currentValue = value;
        if (triggerListeners) listeners.forEach(listener -> listener.accept(value));
    }

    @Override
    protected void periodic() {
        if (currentValue.equals(entry.get())) return;
        currentValue = entry.get();
        listeners.forEach(listener -> listener.accept(currentValue));
    }

    @Override
    public String get() {
        return currentValue;
    }

    @Override
    public void accept(String value) {
        entry.set(value);
    }

    public void addListener(Consumer<String> callback) {
        listeners.add(callback);
    }
}
