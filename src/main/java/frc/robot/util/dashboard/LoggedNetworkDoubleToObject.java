package frc.robot.util.dashboard;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class LoggedNetworkDoubleToObject<T> extends LoggedNetworkInput implements Supplier<T>, Consumer<T> {
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
        entry.set(objectToDouble.apply(value));
        currentValue = value;
    }

    @Override
    protected void periodic() {
        currentValue = doubleToObject.apply(entry.get());
    }

    @Override
    public void accept(T t) {
        set(t);
    }

    @Override
    public T get() {
        return currentValue;
    }
}
