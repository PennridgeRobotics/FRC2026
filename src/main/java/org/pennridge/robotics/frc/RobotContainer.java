package org.pennridge.robotics.frc;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.io.IOException;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import org.pennridge.robotics.frc.subsystems.SwerveSubsystem;
import org.pennridge.robotics.frc.util.enums.Constants.ControllerConstants;

@NullMarked
public class RobotContainer {
    // Initializes subsystems
    private final SwerveSubsystem swerveSubsystem;

    // Initializes controllers
    private final CommandXboxController driverController =
            new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController =
            new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, I/O devices, and commands. */
    public RobotContainer() {
        try {
            swerveSubsystem = new SwerveSubsystem();
        } catch (IOException ex) {
            final var finalException =
                    new RuntimeException("Error instantiating Swerve Subsystem: " + ex.getMessage(), ex);
            DriverStation.reportError(
                    "Error instantiating Swerve Subsystem: " + ex.getMessage(), finalException.getStackTrace());
            throw finalException;
        }

        // autoChooser = AutoBuilder.buildAutoChooser("Epic Auto");
        autoChooser = new SendableChooser<>();
        setupPathPlanner();

        configureBindings();

        // Add the auto chooser to SmartDashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void setupPathPlanner() {}

    public @Nullable Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void periodic() {}

    private void configureBindings() {
        /*swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOrientedCommand(
                driverController::getLeftY, () -> -driverController.getLeftX(), () -> -driverController.getRightX()));

        driverController.start().onTrue(swerveSubsystem.resetYaw());*/

        // for testing
        swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOrientedCommand(
                () -> MathUtil.applyDeadband(driverController.getLeftY(), 0.1),
                () -> MathUtil.applyDeadband(driverController.getLeftX(), 0.1),
                driverController::getRightX,
                driverController::getRightY));
    }

    public void initSmartDashboard() {}
}
