package frc.robot.util.dashboard;

import java.util.Set;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class LoggedNetworkButton extends LoggedNetworkBoolean {
    private final Iterable<Runnable> onPress;

    public LoggedNetworkButton(String rawTopicName, Iterable<Runnable> onPress) {
        super(rawTopicName, false);
        this.onPress = onPress;
    }

    public LoggedNetworkButton(String rawTopicName, Runnable... onPress) {
        this(rawTopicName, Set.of(onPress));
    }

    @Override
    protected void periodic() {
        if (currentValue) {
            set(false);
        }
        super.periodic();
        if (getAsBoolean()) {
            for (Runnable runnable : onPress) {
                runnable.run();
            }
        }
    }
}
