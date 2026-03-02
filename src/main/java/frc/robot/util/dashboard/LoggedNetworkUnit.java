package frc.robot.util.dashboard;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class LoggedNetworkUnit<U extends Unit, T extends Measure<U>> extends LoggedNetworkDoubleToObject<T> {
    @SuppressWarnings("unchecked")
    public LoggedNetworkUnit(String rawTopicName, T defaultValue) {
        super(rawTopicName, defaultValue, num -> (T) defaultValue.unit().of(num), obj -> obj.in(defaultValue.unit()));
    }
}
