package frc.robot.util.dashboard;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class FlashingColorSupplier implements Supplier<String> {
    private boolean isActive;
    private final Timer flashTimer = new Timer();
    private final Time flashInterval;
    private final Color flashColor;

    public FlashingColorSupplier(BooleanSupplier isActiveSupplier, Color flashColor, Time flashInterval) {
        this.flashInterval = flashInterval;
        this.flashColor = flashColor;
        final var trigger = new Trigger(isActiveSupplier);
        trigger.onTrue(Commands.runOnce(() -> {
            flashTimer.restart();
            isActive = true;
        }));
        trigger.onFalse(Commands.runOnce(() -> {
            isActive = false;
            flashTimer.reset();
        }));
    }

    @Override
    public String get() {
        if (!isActive) return Color.kBlack.toHexString();
        final var currentTime = flashTimer.get();
        final var flashInterval = this.flashInterval.in(Seconds);
        final var shouldFlash = (currentTime % flashInterval) < flashInterval / 2;
        return (shouldFlash ? flashColor : Color.kBlack).toHexString();
    }
}
