package frc.robot.util.dashboard;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class LoggedNetworkDoubleToObject<T> extends LoggedNetworkInput implements Supplier<T>, Consumer<T> {
    private final List<Consumer<T>> listeners = new ArrayList<>();
    private final DoubleEntry entry;
    private final Function<T, Double> objectToDouble;
    private final Function<Double, T> doubleToObject;
    private T currentValue;

    public LoggedNetworkDoubleToObject(
            String rawTopicName,
            T defaultValue,
            Function<Double, T> doubleToObject,
            Function<T, Double> objectToDouble) {
        super(rawTopicName);
        this.objectToDouble = objectToDouble;
        this.doubleToObject = doubleToObject;
        entry = NetworkTableInstance.getDefault()
                .getDoubleTopic(topicName)
                .getEntry(objectToDouble.apply(defaultValue));
        entry.set(objectToDouble.apply(defaultValue));
        currentValue = doubleToObject.apply(entry.get());
    }

    public void set(T value) {
        set(value, true);
    }

    public void set(T value, boolean triggerListeners) {
        if (currentValue.equals(value)) return;
        entry.set(objectToDouble.apply(value));
        currentValue = value;
        if (triggerListeners) listeners.forEach(listener -> listener.accept(value));
    }

    @Override
    protected void periodic() {
        if (currentValue.equals(doubleToObject.apply(entry.get()))) return;
        currentValue = doubleToObject.apply(entry.get());
        listeners.forEach(listener -> listener.accept(currentValue));
    }

    @Override
    public void accept(T t) {
        set(t);
    }

    @Override
    public T get() {
        return currentValue;
    }

    public void addListener(Consumer<T> callback) {
        listeners.add(callback);
    }
}
