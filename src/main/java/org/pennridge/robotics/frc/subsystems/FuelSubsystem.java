package org.pennridge.robotics.frc.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jspecify.annotations.NullMarked;
import org.pennridge.robotics.frc.util.enums.Constants.FuelConstants;

@NullMarked
public class FuelSubsystem extends SubsystemBase {
    private final VictorSPX intakeLauncherRoller;
    private final VictorSPX feederRoller;

    public FuelSubsystem() {
        intakeLauncherRoller = new VictorSPX(FuelConstants.INTAKE_LAUNCHER_MOTOR_ID);
        feederRoller = new VictorSPX(FuelConstants.FEEDER_MOTOR_ID);
    }

    private void stop() {}
}
