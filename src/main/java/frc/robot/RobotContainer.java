package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.classes.AutoManager;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.ShooterCalculator;
import frc.robot.util.dashboard.LoggedNetworkInput;
import frc.robot.util.dashboard.MultiMotorInfoSendable;
import frc.robot.util.enums.Constants.*;
import frc.robot.util.enums.PositionCalibrationLocation;
import frc.robot.util.enums.SpeedMultiplier;
import java.io.IOException;
import java.util.Map;
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
    private final PowerDistribution powerDistribution;
    private final MultiMotorInfoSendable motorInfo = new MultiMotorInfoSendable();
    private final AutoManager autoManager;

    // Initializes controllers
    private final CommandXboxController driverController =
            new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final @Nullable CommandXboxController operatorController = ControllerConstants.OPERATOR_ENABLED
            ? new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT)
            : null;

    private final SendableChooser<Command> autoChooser;

    private boolean useOdometry = true;
    private final Trigger useOdometryTrigger = new Trigger(() -> useOdometry);

    /** The container for the robot. Contains subsystems, I/O devices, and commands. */
    public RobotContainer() {
        powerDistribution =
                new PowerDistribution(MiscConstants.POWER_DISTRIBUTION_HUB_ID, PowerDistribution.ModuleType.kRev);
        try {
            swerveSubsystem = new SwerveSubsystem(motorInfo);
        } catch (IOException ex) {
            final var finalException =
                    new RuntimeException("Error instantiating Swerve Subsystem: " + ex.getMessage(), ex);
            DriverStation.reportError(
                    "Error instantiating Swerve Subsystem: " + ex.getMessage(), finalException.getStackTrace());
            throw finalException;
        }
        shooterCalculator = new ShooterCalculator(swerveSubsystem::getRobotPose);
        fuelSubsystem = FuelConstants.FUEL_SUBSYSTEM_ENABLED ? new FuelSubsystem(shooterCalculator, motorInfo) : null;
        climberSubsystem = ClimberConstants.CLIMBER_ENABLED ? new ClimberSubsystem(motorInfo) : null;
        lightsSubsystem = LightConstants.LIGHTS_ENABLED
                ? new LightsSubsystem(swerveSubsystem, fuelSubsystem, climberSubsystem)
                : null;

        // autoChooser = AutoBuilder.buildAutoChooser("Epic Auto");
        autoChooser = new SendableChooser<>();
        setupPathPlanner();
        autoManager = new AutoManager(swerveSubsystem, swerveSubsystem.getPathBuilder(), fuelSubsystem);

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
                driverController.rightStick().whileTrue(swerveSubsystem.lockYawTowardsVelocity());
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

        driverController.leftTrigger().onTrue(swerveSubsystem.setSpeedMultiplierCommand(SpeedMultiplier.SLOW));
        driverController
                .leftTrigger()
                .negate()
                .and(driverController.rightTrigger().negate())
                .onTrue(swerveSubsystem.setSpeedMultiplierCommand(SpeedMultiplier.NORMAL));
        driverController.rightTrigger().onTrue(swerveSubsystem.setSpeedMultiplierCommand(SpeedMultiplier.FAST));
        driverController.start().onTrue(new InstantCommand(swerveSubsystem::zeroGyroWithAlliance));
        driverController.y().whileTrue(swerveSubsystem.faceTowardsHubCommand());
        if (fuelSubsystem != null) {
            driverController.b().whileTrue(fuelSubsystem.windUpAndLaunchCommand());
            driverController.a().whileTrue(fuelSubsystem.intakeCommand());
        }
        /*driverController
        .a()
        .whileTrue(swerveSubsystem.resetPoseFromCalibrationPosition(
                PositionCalibrationLocation.FRONT_LEFT_OF_HUB));*/

        if (operatorController == null) {
            return;
        }
        // operatorController.start().whileTrue(swerveSubsystem.straightenWheelsCommand());
        operatorController
                .start()
                .and(operatorController.leftTrigger())
                .whileTrue(swerveSubsystem.resetPoseFromCalibrationPosition(
                        PositionCalibrationLocation.FRONT_LEFT_OF_HUB));
        operatorController
                .start()
                .and(operatorController.leftBumper())
                .whileTrue(swerveSubsystem.resetPoseFromCalibrationPosition(
                        PositionCalibrationLocation.LEFT_DEPOT_CORNER));
        operatorController
                .start()
                .and(operatorController.rightTrigger())
                .whileTrue(swerveSubsystem.resetPoseFromCalibrationPosition(
                        PositionCalibrationLocation.FRONT_RIGHT_OF_HUB));
        operatorController
                .start()
                .and(operatorController.rightBumper())
                .whileTrue(swerveSubsystem.resetPoseFromCalibrationPosition(
                        PositionCalibrationLocation.RIGHT_OUTPOST_CORNER));
        operatorController.x().whileTrue(swerveSubsystem.enableManualBumpLock());

        if (fuelSubsystem != null) {
            operatorController
                    .rightBumper()
                    .and(operatorController.start().negate())
                    .whileTrue(fuelSubsystem.windUpCommand());
            operatorController
                    .rightTrigger()
                    .and(operatorController.start().negate())
                    .whileTrue(Commands.select(
                                    Map.of(
                                            false, fuelSubsystem.windUpAndLaunchCommand(),
                                            true, fuelSubsystem.launchCommand()),
                                    operatorController.leftTrigger()::getAsBoolean // force launch
                                    )
                            .finallyDo(() -> CommandScheduler.getInstance()
                                    .schedule(fuelSubsystem
                                            .unjamCommand()
                                            .withTimeout(FuelConstants.UNJAM_AFTER_LAUNCH_TIME))));
            operatorController.leftBumper().whileTrue(fuelSubsystem.ejectCommand());
            operatorController.a().whileTrue(fuelSubsystem.intakeCommand());
            operatorController.b().whileTrue(fuelSubsystem.unjamCommand());
        }
    }

    public void initSmartDashboard() {
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Power Distribution", powerDistribution);
        SmartDashboard.putData(
                "RobotContainer",
                builder -> builder.addBooleanProperty("Use Odometry", () -> useOdometry, v -> useOdometry = v));
        SmartDashboard.putData("Motor Info", motorInfo);
    }

    public void preSchedulerUpdate() {
        shooterCalculator.prePeriodic();
        LoggedNetworkInput.runAllPeriodic();
    }

    public void postSchedulerUpdate() {
        NetworkTableInstance.getDefault().flush();
    }
}
