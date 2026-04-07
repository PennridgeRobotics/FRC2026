package frc.robot.util.dashboard;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class LoggedNetworkUnit<U extends Unit, T extends Measure<U>> extends LoggedNetworkDoubleToObject<T> {
    public LoggedNetworkUnit(String rawTopicName, T defaultValue) {
        this(rawTopicName, defaultValue, defaultValue.unit());
    }

    @SuppressWarnings("unchecked")
    public LoggedNetworkUnit(String rawTopicName, T defaultValue, U unit) {
        super(rawTopicName, defaultValue, num -> (T) unit.of(num), obj -> obj.in(unit));
    }

    public LoggedNetworkUnit(String rawTopicName, Supplier<T> supplier) {
        this(rawTopicName, supplier.get(), supplier.get().unit());
    }

    public LoggedNetworkUnit(String rawTopicName, Supplier<T> supplier, U unit) {
        this(rawTopicName, supplier.get(), unit);
        setSupplier(supplier);
    }
}
