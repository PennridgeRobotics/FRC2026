package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.ShooterCalculator;
import frc.robot.util.enums.Constants.ClimberConstants;
import frc.robot.util.enums.Constants.ControllerConstants;
import frc.robot.util.enums.Constants.FuelConstants;
import frc.robot.util.enums.Constants.LightConstants;
import java.io.IOException;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;

@NullMarked
public class RobotContainer {
    // Initializes subsystems
    private final SwerveSubsystem swerveSubsystem;
    private final @Nullable LightsSubsystem lightsSubsystem;
    private final @Nullable FuelSubsystem fuelSubsystem;
    private final @Nullable ClimberSubsystem climberSubsystem;
    // private final @Nullable TankSubsystem tankSubsystem;

    private final ShooterCalculator shooterCalculator;

    // Initializes controllers
    private final CommandXboxController driverController =
            new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final @Nullable CommandXboxController operatorController = ControllerConstants.OPERATOR_ENABLED
            ? new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT)
            : null;

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
        shooterCalculator = new ShooterCalculator(swerveSubsystem::getPose);
        fuelSubsystem = FuelConstants.FUEL_SUBSYSTEM_ENABLED ? new FuelSubsystem(shooterCalculator) : null;
        climberSubsystem = ClimberConstants.CLIMBER_ENABLED ? new ClimberSubsystem() : null;
        lightsSubsystem = LightConstants.LIGHTS_ENABLED
                ? new LightsSubsystem(swerveSubsystem, fuelSubsystem, climberSubsystem)
                : null;

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
        final var fieldOriented = true;
        final var forceRobotOrientedRotation = true;
        if (fieldOriented) {
            if (forceRobotOrientedRotation && operatorController != null) {
                swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldAndRobotOrientedCommand(
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        () -> -operatorController.getLeftX(),
                        () -> -operatorController.getLeftY()));
            } else {
                swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOrientedHeadingCommand(
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        () -> -driverController.getRightY()));
            }
        } else {
            swerveSubsystem.setDefaultCommand(swerveSubsystem.driveRobotOrientedCommand(
                    () -> MathUtil.applyDeadband(-driverController.getLeftY(), 0.1),
                    () -> MathUtil.applyDeadband(-driverController.getLeftX(), 0.1),
                    () -> -driverController.getRightX()));
        }

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

        driverController.start().onTrue(new InstantCommand(swerveSubsystem::zeroGyroWithAlliance));

        if (operatorController != null) {
            operatorController.start().whileTrue(swerveSubsystem.straightenWheelsCommand());
            operatorController.x().onTrue(swerveSubsystem.setManualBumpLock(true));
            operatorController.x().onFalse(swerveSubsystem.setManualBumpLock(false));
            if (fuelSubsystem != null) {
                operatorController.a().onTrue(fuelSubsystem.addCurrentDataToShooterMap());
            }
        }
    }

    public void initSmartDashboard() {}
}
