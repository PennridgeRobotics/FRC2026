package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.ShooterCalculator;
import frc.robot.util.enums.Constants.ClimberConstants;
import frc.robot.util.enums.Constants.ControllerConstants;
import frc.robot.util.enums.Constants.FuelConstants;
import frc.robot.util.enums.Constants.LightConstants;
import frc.robot.util.enums.PositionCalibrationLocation;
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

    private boolean useOdometry = true;
    private final Trigger useOdometryTrigger = new Trigger(() -> useOdometry);

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
        shooterCalculator = new ShooterCalculator(swerveSubsystem::getRobotPose);
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
        SmartDashboard.putData(
                "RobotContainer",
                builder -> builder.addBooleanProperty("Use Odometry", () -> useOdometry, v -> useOdometry = v));
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

        // driverController.start().onTrue(new InstantCommand(swerveSubsystem::zeroGyroWithAlliance));
        driverController.y().whileTrue(swerveSubsystem.faceTowardsHubCommand());

        if (operatorController != null) {
            operatorController.start().whileTrue(swerveSubsystem.straightenWheelsCommand());
            operatorController
                    .leftTrigger()
                    .whileTrue(swerveSubsystem.resetPoseFromCalibrationPosition(
                            PositionCalibrationLocation.LEFT_TRENCH_OUTER));
            operatorController
                    .leftBumper()
                    .whileTrue(swerveSubsystem.resetPoseFromCalibrationPosition(
                            PositionCalibrationLocation.LEFT_DEPOT_CORNER));
            operatorController
                    .rightTrigger()
                    .whileTrue(swerveSubsystem.resetPoseFromCalibrationPosition(
                            PositionCalibrationLocation.RIGHT_TRENCH_OUTER));
            operatorController
                    .rightBumper()
                    .whileTrue(swerveSubsystem.resetPoseFromCalibrationPosition(
                            PositionCalibrationLocation.RIGHT_OUTPOST_CORNER));
            operatorController.x().whileTrue(swerveSubsystem.enableManualBumpLock());
        }
    }

    public void initSmartDashboard() {}

    public void preSchedulerUpdate() {
        shooterCalculator.clearShotCache();
    }

    public void postSchedulerUpdate() {
        NetworkTableInstance.getDefault().flush();
    }
}
