package org.pennridge.robotics.frc.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class SwerveSubsystem extends SubsystemBase {

    public SwerveSubsystem() {
        initSmartDashboard();
    }

    private void initSmartDashboard() {}

    @Override
    public void periodic() {}
}
