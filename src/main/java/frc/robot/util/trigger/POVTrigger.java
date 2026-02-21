package frc.robot.util.trigger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.enums.Direction;
import java.util.function.DoubleSupplier;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class POVTrigger extends Trigger {
    public POVTrigger(final DoubleSupplier angleInput, final Direction direction) {
        super(() -> direction
                == Direction.getClosestCompassDirection(Rotation2d.fromDegrees(-angleInput.getAsDouble()), true));
    }
}
