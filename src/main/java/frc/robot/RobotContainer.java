package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.dashboard.CANBusLoadSendable;
import frc.robot.util.enums.Constants.*;
import java.io.IOException;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;

@NullMarked
public class RobotContainer {
    // Initializes subsystems
    private final @Nullable LightsSubsystem lightsSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final @Nullable FuelSubsystem fuelSubsystem;
    private final @Nullable ClimberSubsystem climberSubsystem;

    // Initializes controllers
    private final CommandXboxController driverController =
            new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    /*private final CommandXboxController operatorController =
    new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);*/

    private final SendableChooser<Command> autoChooser;
    private final CANBusLoadSendable canBusLoadSendable;

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
        fuelSubsystem = FuelConstants.FUEL_SUBSYSTEM_ENABLED ? new FuelSubsystem() : null;
        climberSubsystem = ClimberConstants.CLIMBER_ENABLED ? new ClimberSubsystem() : null;
        lightsSubsystem = LightConstants.LIGHTS_ENABLED
                ? new LightsSubsystem(swerveSubsystem, fuelSubsystem, climberSubsystem)
                : null;

        // autoChooser = AutoBuilder.buildAutoChooser("Epic Auto");
        autoChooser = new SendableChooser<>();
        canBusLoadSendable = new CANBusLoadSendable();
        setupPathPlanner();

        configureBindings();

        initSmartDashboard();
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
        swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOrientedHeadingCommand(
                () -> MathUtil.applyDeadband(driverController.getLeftY(), 0.1),
                () -> MathUtil.applyDeadband(driverController.getLeftX(), 0.1),
                driverController::getRightX,
                driverController::getRightY));
        driverController
                .y()
                .whileTrue(swerveSubsystem.driveFieldOrientedCommand(
                        () -> MetersPerSecond.of(0.5), MetersPerSecond::zero, DegreesPerSecond::zero));
        driverController
                .b()
                .whileTrue(swerveSubsystem.driveFieldOrientedHeadingCommand(
                        MetersPerSecond::zero, MetersPerSecond::zero, () -> Rotation2d.kCW_90deg));
        driverController
                .x()
                .whileTrue(swerveSubsystem.driveFieldOrientedHeadingCommand(
                        MetersPerSecond::zero, MetersPerSecond::zero, () -> Rotation2d.kCCW_90deg));
        driverController
                .a()
                .whileTrue(swerveSubsystem.driveFieldOrientedHeadingCommand(
                        MetersPerSecond::zero, MetersPerSecond::zero, () -> Rotation2d.k180deg));
    }

    public void initSmartDashboard() {
        // Add the auto chooser to SmartDashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);
        // Add the CAN Bus Load Sendable to SmartDashboard
        SmartDashboard.putData("CAN Bus Load", canBusLoadSendable);
    }
}
