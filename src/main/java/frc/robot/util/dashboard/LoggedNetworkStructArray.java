package frc.robot.util.dashboard;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayEntry;
import edu.wpi.first.util.struct.Struct;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;

@NullMarked
public class LoggedNetworkStructArray<T> extends LoggedNetworkInput implements Supplier<T[]>, Consumer<T[]> {
    private final List<Consumer<T[]>> listeners = new ArrayList<>();
    private final StructArrayEntry<T> entry;
    private T[] currentValue;
    private @Nullable Supplier<T[]> supplier;

    public LoggedNetworkStructArray(String rawTopicName, Struct<T> struct, T[] defaultValue) {
        super(rawTopicName);
        entry = NetworkTableInstance.getDefault()
                .getStructArrayTopic(topicName, struct)
                .getEntry(defaultValue);
        entry.set(defaultValue);
        currentValue = entry.get();
    }

    public LoggedNetworkStructArray(String rawTopicName, Struct<T> struct, Supplier<T[]> supplier) {
        this(rawTopicName, struct, supplier.get());
        this.supplier = supplier;
    }

    public void set(T[] value) {
        set(value, false);
    }

    public void set(T[] value, boolean triggerListeners) {
        if (Arrays.equals(currentValue, value)) return;
        entry.set(value);
        currentValue = value;
        if (triggerListeners) listeners.forEach(listener -> listener.accept(value));
    }

    @Override
    protected void periodic() {
        if (Arrays.equals(currentValue, entry.get())) {
            if (supplier != null) set(supplier.get());
            return;
        }
        currentValue = entry.get();
        listeners.forEach(listener -> listener.accept(currentValue));
    }

    @Override
    public T[] get() {
        return currentValue;
    }

    @Override
    public void accept(T[] value) {
        entry.set(value);
    }

    public void addListener(Consumer<T[]> callback) {
        listeners.add(callback);
    }

    public void setSupplier(@Nullable Supplier<T[]> supplier) {
        this.supplier = supplier;
    }
}
