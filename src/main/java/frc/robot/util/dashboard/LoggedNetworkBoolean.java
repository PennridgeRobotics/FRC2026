package frc.robot.util.dashboard;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class LoggedNetworkBoolean extends LoggedNetworkInput implements BooleanSupplier, BooleanConsumer {
    private final List<BooleanConsumer> listeners = new ArrayList<>();
    private final BooleanEntry entry;
    protected boolean currentValue;

    public LoggedNetworkBoolean(String rawTopicName, boolean defaultValue) {
        super(rawTopicName);
        entry = NetworkTableInstance.getDefault().getBooleanTopic(topicName).getEntry(defaultValue);
        entry.set(defaultValue);
        currentValue = entry.get();
    }

    public void set(boolean value) {
        set(value, true);
    }

    public void set(boolean value, boolean triggerListeners) {
        if (currentValue == value) return;
        entry.set(value);
        currentValue = value;
        if (triggerListeners) listeners.forEach(listener -> listener.accept(value));
    }

    @Override
    protected void periodic() {
        if (currentValue == entry.get()) return;
        currentValue = entry.get();
        listeners.forEach(listener -> listener.accept(currentValue));
    }

    @Override
    public boolean getAsBoolean() {
        return currentValue;
    }

    @Override
    public void accept(boolean value) {
        set(value);
    }

    public Trigger getTrigger() {
        return new Trigger(this);
    }

    public void addListener(BooleanConsumer callback) {
        listeners.add(callback);
    }
}
