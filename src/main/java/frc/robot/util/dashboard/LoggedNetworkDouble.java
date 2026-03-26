package frc.robot.util.dashboard;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;

@NullMarked
public class LoggedNetworkDouble extends LoggedNetworkInput implements DoubleSupplier, DoubleConsumer {
    private final List<DoubleConsumer> listeners = new ArrayList<>();
    private final DoubleEntry entry;
    private double currentValue;
    private @Nullable DoubleSupplier supplier;

    public LoggedNetworkDouble(String rawTopicName, double defaultValue) {
        super(rawTopicName);
        entry = NetworkTableInstance.getDefault().getDoubleTopic(topicName).getEntry(defaultValue);
        entry.set(defaultValue);
        currentValue = entry.get();
    }

    public LoggedNetworkDouble(String rawTopicName, DoubleSupplier supplier) {
        this(rawTopicName, supplier.getAsDouble());
        this.supplier = supplier;
    }

    public void set(double value) {
        set(value, false);
    }

    public void set(double value, boolean triggerListeners) {
        if (currentValue == value) return;
        entry.set(value);
        currentValue = value;
        if (triggerListeners) listeners.forEach(listener -> listener.accept(value));
    }

    @Override
    protected void periodic() {
        if (currentValue == entry.get()) {
            if (supplier != null) set(supplier.getAsDouble());
            return;
        }
        currentValue = entry.get();
        listeners.forEach(listener -> listener.accept(currentValue));
    }

    @Override
    public double getAsDouble() {
        return currentValue;
    }

    @Override
    public void accept(double value) {
        set(value);
    }

    public void addListener(DoubleConsumer callback) {
        listeners.add(callback);
    }

    public void setSupplier(@Nullable DoubleSupplier supplier) {
        this.supplier = supplier;
    }
}
