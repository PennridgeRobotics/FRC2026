package org.pennridge.robotics.frc.util.trigger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.jspecify.annotations.NullMarked;
import org.pennridge.robotics.frc.util.enums.Direction;

@NullMarked
public class POVTrigger extends Trigger {
    public POVTrigger(final DoubleSupplier angleInput, final Direction direction) {
        super(() -> direction
                == Direction.getClosestCompassDirection(Rotation2d.fromDegrees(-angleInput.getAsDouble()), true));
    }
}
