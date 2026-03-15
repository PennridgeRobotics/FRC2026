package frc.robot.util.dashboard;

import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.List;
import java.util.function.IntSupplier;
import java.util.function.LongConsumer;
import java.util.function.LongSupplier;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class LoggedNetworkInteger extends LoggedNetworkInput implements LongSupplier, LongConsumer, IntSupplier {
    private final List<LongConsumer> listeners = new ArrayList<>();
    private final IntegerEntry entry;
    private long currentValue;

    public LoggedNetworkInteger(String rawTopicName, long defaultValue) {
        super(rawTopicName);
        entry = NetworkTableInstance.getDefault().getIntegerTopic(topicName).getEntry(defaultValue);
        entry.set(defaultValue);
        currentValue = entry.get();
    }

    public void set(long value) {
        set(value, true);
    }

    public void set(long value, boolean triggerListeners) {
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
    public long getAsLong() {
        return currentValue;
    }

    @Override
    public int getAsInt() {
        if (currentValue > Integer.MAX_VALUE) set(Integer.MAX_VALUE);
        if (currentValue < Integer.MIN_VALUE) set(Integer.MIN_VALUE);
        return (int) currentValue;
    }

    @Override
    public void accept(long value) {
        set(value);
    }

    public void addListener(LongConsumer callback) {
        listeners.add(callback);
    }
}
