package frc.robot.util.dashboard;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.util.struct.Struct;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class LoggedNetworkStruct<T> extends LoggedNetworkInput implements Supplier<T>, Consumer<T> {
    private final List<Consumer<T>> listeners = new ArrayList<>();
    private final StructEntry<T> entry;
    private T currentValue;

    public LoggedNetworkStruct(String rawTopicName, Struct<T> struct, T defaultValue) {
        super(rawTopicName);
        entry = NetworkTableInstance.getDefault()
                .getStructTopic(topicName, struct)
                .getEntry(defaultValue);
        entry.set(defaultValue);
        currentValue = entry.get();
    }

    public void set(T value) {
        set(value, false);
    }

    public void set(T value, boolean triggerListeners) {
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
    public T get() {
        return currentValue;
    }

    @Override
    public void accept(T value) {
        entry.set(value);
    }

    public void addListener(Consumer<T> callback) {
        listeners.add(callback);
    }
}
