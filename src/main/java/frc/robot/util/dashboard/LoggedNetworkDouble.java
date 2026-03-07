package frc.robot.util.dashboard;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class LoggedNetworkDouble extends LoggedNetworkInput implements DoubleSupplier, DoubleConsumer {
    private final DoubleEntry entry;
    private double currentValue;

    public LoggedNetworkDouble(String rawTopicName, double defaultValue) {
        super(rawTopicName);
        entry = NetworkTableInstance.getDefault().getDoubleTopic(topicName).getEntry(defaultValue);
        entry.set(defaultValue);
        currentValue = entry.get();
    }

    public void set(double value) {
        entry.set(value);
        currentValue = value;
    }

    @Override
    protected void periodic() {
        currentValue = entry.get();
    }

    @Override
    public double getAsDouble() {
        return currentValue;
    }

    @Override
    public void accept(double value) {
        set(value);
    }
}
