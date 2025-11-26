package org.pennridge.robotics.frc.util.trigger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.jetbrains.annotations.NotNull;
import org.pennridge.robotics.frc.util.enums.Direction;

public class POVTrigger extends Trigger {
    public POVTrigger(final @NotNull DoubleSupplier angleInput, final @NotNull Direction direction) {
        super(() -> direction
                == Direction.getClosestCompassDirection(Rotation2d.fromDegrees(-angleInput.getAsDouble()), true));
    }
}
