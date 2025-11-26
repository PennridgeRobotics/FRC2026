package org.pennridge.robotics.frc;

import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.pennridge.robotics.frc.util.enums.Constants.ControllerConstants;

public class RobotContainer {
    // Initializes subsystems

    // Initializes controllers
    private final @NotNull CommandXboxController driverController =
            new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final @NotNull CommandXboxController operatorController =
            new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

    private final @NotNull SendableChooser<@NotNull Command> autoChooser;
    private final @NotNull AHRS ahrs;

    /** The container for the robot. Contains subsystems, I/O devices, and commands. */
    public RobotContainer() {
        // Creates new AHRS NavX object for gyro
        try {
            ahrs = new AHRS(AHRS.NavXComType.kUSB1);
            ahrs.enableLogging(true);
            ahrs.zeroYaw();
            System.out.println("====== SETTING UP NAVX ======");
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(), true);
            throw ex;
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

    public @NotNull AHRS getAHRS() {
        return ahrs;
    }

    public void periodic() {}

    private void configureBindings() {}

    public void initSmartDashboard() {}
}
