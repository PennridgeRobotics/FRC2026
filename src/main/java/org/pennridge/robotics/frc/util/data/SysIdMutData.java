package org.pennridge.robotics.frc.util.data;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import org.jetbrains.annotations.NotNull;

public record SysIdMutData(
        @NotNull MutVoltage appliedVoltage,
        @NotNull MutDistance distance,
        @NotNull MutLinearVelocity velocity,
        @NotNull MutAngle rotations) {
    public SysIdMutData() {
        this(Volts.mutable(0), Meters.mutable(0), MetersPerSecond.mutable(0), Rotations.mutable(0));
    }
}
