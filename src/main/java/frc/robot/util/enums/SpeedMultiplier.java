package frc.robot.util.enums;

import org.jspecify.annotations.NullMarked;

@NullMarked
public enum SpeedMultiplier {
    SLOW(0.3),
    NORMAL(0.7),
    FAST(1.0),
    ;
    private final double multiplier;

    SpeedMultiplier(double multiplier) {
        this.multiplier = multiplier;
    }

    public double getMultiplier() {
        return multiplier;
    }
}
