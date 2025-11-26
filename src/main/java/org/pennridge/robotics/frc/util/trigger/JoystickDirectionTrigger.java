package org.pennridge.robotics.frc.util.trigger;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.pennridge.robotics.frc.util.enums.Direction;

public class JoystickDirectionTrigger extends Trigger {
    private static final double JOYSTICK_MIN = 0.6;

    public JoystickDirectionTrigger(
            final @NotNull DoubleSupplier xInput,
            final @NotNull DoubleSupplier yInput,
            final @NotNull Direction direction) {
        super(() -> direction == getDirection(xInput.getAsDouble(), -yInput.getAsDouble()));
    }

    public static @Nullable Direction getDirection(final double xInput, final double yInput) {
        if (Math.abs(xInput) < JOYSTICK_MIN && Math.abs(yInput) < JOYSTICK_MIN) {
            return null;
        }

        final int xStatus = xInput > JOYSTICK_MIN ? 1 : xInput < -JOYSTICK_MIN ? -1 : 0;
        final int yStatus = yInput > JOYSTICK_MIN ? 1 : yInput < -JOYSTICK_MIN ? -1 : 0;

        if (xStatus == 0 && yStatus == 0) {
            return null;
        }

        return Direction.fromXY(xStatus, yStatus);
    }
}
