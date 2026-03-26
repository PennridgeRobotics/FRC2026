package frc.robot.util.dashboard;

import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.List;
import java.util.function.IntSupplier;
import java.util.function.LongConsumer;
import java.util.function.LongSupplier;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;

@NullMarked
public class LoggedNetworkInteger extends LoggedNetworkInput implements LongSupplier, LongConsumer, IntSupplier {
    private final List<LongConsumer> listeners = new ArrayList<>();
    private final IntegerEntry entry;
    private long currentValue;
    private @Nullable LongSupplier supplier;

    public LoggedNetworkInteger(String rawTopicName, long defaultValue) {
        super(rawTopicName);
        entry = NetworkTableInstance.getDefault().getIntegerTopic(topicName).getEntry(defaultValue);
        entry.set(defaultValue);
        currentValue = entry.get();
    }

    public LoggedNetworkInteger(String rawTopicName, LongSupplier supplier) {
        this(rawTopicName, supplier.getAsLong());
        this.supplier = supplier;
    }

    public LoggedNetworkInteger(String rawTopicName, IntSupplier supplier) {
        this(rawTopicName, () -> (long) supplier.getAsInt());
    }

    public void set(long value) {
        set(value, false);
    }

    public void set(long value, boolean triggerListeners) {
        if (currentValue == value) return;
        entry.set(value);
        currentValue = value;
        if (triggerListeners) listeners.forEach(listener -> listener.accept(value));
    }

    @Override
    protected void periodic() {
        if (currentValue == entry.get()) {
            if (supplier != null) set(supplier.getAsLong());
            return;
        }
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

    public void setSupplier(@Nullable LongSupplier supplier) {
        this.supplier = supplier;
    }
}
