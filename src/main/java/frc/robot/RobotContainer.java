package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.FuelSubsystem.OperatorFuelRequest;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.AutoManager;
import frc.robot.util.HubTracker;
import frc.robot.util.ShooterCalculator;
import frc.robot.util.StringUtils;
import frc.robot.util.dashboard.Field2dElastic;
import frc.robot.util.dashboard.LoggedNetworkButton;
import frc.robot.util.dashboard.LoggedNetworkInput;
import frc.robot.util.dashboard.MultiMotorInfoSendable;
import frc.robot.util.enums.Constants.ClimberConstants;
import frc.robot.util.enums.Constants.ControllerConstants;
import frc.robot.util.enums.Constants.FuelConstants;
import frc.robot.util.enums.Constants.LightConstants;
import frc.robot.util.enums.Constants.MiscConstants;
import frc.robot.util.enums.PositionCalibrationLocation;
import frc.robot.util.enums.SpeedMultiplier;
import java.io.IOException;
import java.util.List;
import java.util.Set;
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
    private final @Nullable AutoManager autoManager;
    private final Field2dElastic field = new Field2dElastic();

    private final StructPublisher<Pose2d> aheadRobotPose;
    private final StructPublisher<Pose2d> behindRobotPose;

    // Initializes controllers
    private final CommandXboxController driverController =
            new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController =
            new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

    private final SendableChooser<AutoManager.AutoStartLocation> autoStartLocationChooser;
    private boolean autoClimb = false;
    private boolean autoDepot = false;
    private boolean autoOutpost = false;

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
        shooterCalculator = swerveSubsystem.getShooterCalculator();
        fuelSubsystem = FuelConstants.FUEL_SUBSYSTEM_ENABLED ? new FuelSubsystem(shooterCalculator, motorInfo) : null;
        climberSubsystem = ClimberConstants.CLIMBER_ENABLED ? new ClimberSubsystem(motorInfo) : null;
        lightsSubsystem = LightConstants.LIGHTS_ENABLED
                ? new LightsSubsystem(swerveSubsystem, fuelSubsystem, climberSubsystem)
                : null;

        // autoChooser = AutoBuilder.buildAutoChooser("Epic Auto");
        autoStartLocationChooser = new SendableChooser<>();
        for (final var location : AutoManager.AutoStartLocation.values()) {
            autoStartLocationChooser.addOption(StringUtils.capitalizeFully(location.name()), location);
        }

        autoManager = ((fuelSubsystem != null) && (climberSubsystem != null))
                ? new AutoManager(swerveSubsystem, swerveSubsystem.getPathBuilder(), fuelSubsystem, climberSubsystem)
                : null;

        aheadRobotPose = NetworkTableInstance.getDefault()
                .getStructTopic("Robot Pose Ahead", Pose2d.struct)
                .publish();
        behindRobotPose = NetworkTableInstance.getDefault()
                .getStructTopic("Robot Pose Behind", Pose2d.struct)
                .publish();

        configureBindings();

        initSmartDashboard();

        HubTracker.isActive(); // initialize HubTracker
    }

    public @Nullable Command getAutonomousCommand() {
        updateSmartDashboard();
        return autoManager != null
                ? autoManager.getAutoCommand(new AutoManager.AutoOptions(
                        autoStartLocationChooser.getSelected(), autoDepot, autoOutpost, autoClimb))
                : null;
    }

    public void periodic() {
        updateField();
        updateAheadRobotPose();
    }

    private void updateAheadRobotPose() {
        aheadRobotPose.set(swerveSubsystem.getRobotPose().plus(new Transform2d(100.0, 0, Rotation2d.kZero)));
        behindRobotPose.set(swerveSubsystem.getRobotPose().plus(new Transform2d(-100.0, 0, Rotation2d.kZero)));
    }

    private void configureBindings() {
        /*swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOrientedCommand(
                driverController::getLeftY, () -> -driverController.getLeftX(), () -> -driverController.getRightX()));

        driverController.start().onTrue(swerveSubsystem.resetYaw());*/

        // for testing
        final var fieldOriented = true;
        final var forceRobotOrientedRotation = true;
        if (fieldOriented) {
            if (forceRobotOrientedRotation) {
                swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldAndRobotOrientedCommand(
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        () -> /*-operatorController.getLeftX()*/ 0,
                        () -> /*-operatorController.getLeftY()*/ 0));
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

        driverController.leftBumper().onTrue(swerveSubsystem.setSpeedMultiplierCommand(SpeedMultiplier.SLOW));
        driverController
                .leftBumper()
                .negate()
                .and(driverController.rightTrigger().negate())
                .onTrue(swerveSubsystem.setSpeedMultiplierCommand(SpeedMultiplier.NORMAL));
        driverController.rightTrigger().onTrue(swerveSubsystem.setSpeedMultiplierCommand(SpeedMultiplier.FAST));
        driverController.start().onTrue(new InstantCommand(swerveSubsystem::zeroGyroWithAlliance));
        driverController.y().whileTrue(swerveSubsystem.faceTowardsHubCommand());
        driverController.x().whileTrue(swerveSubsystem.lockPoseCommand());
        if (fuelSubsystem != null) {
            driverController
                    .b()
                    .whileTrue(fuelSubsystem.launchCommand(true))
                    .and(shooterCalculator::isUsingSOTM)
                    .whileTrue(swerveSubsystem.faceTowardsHubCommand());
            driverController.leftTrigger().whileTrue(fuelSubsystem.intakeCommand());
        }
        /*driverController
        .a()
        .whileTrue(swerveSubsystem.resetPoseFromCalibrationPosition(
                PositionCalibrationLocation.FRONT_LEFT_OF_HUB));*/

        // operatorController.start().whileTrue(swerveSubsystem.straightenWheelsCommand());
        final var calibrations = List.of(
                new Pair<>(operatorController.leftTrigger(), PositionCalibrationLocation.FRONT_LEFT_OF_HUB),
                new Pair<>(operatorController.leftBumper(), PositionCalibrationLocation.LEFT_DEPOT_CORNER),
                new Pair<>(operatorController.rightTrigger(), PositionCalibrationLocation.FRONT_RIGHT_OF_HUB),
                new Pair<>(operatorController.rightBumper(), PositionCalibrationLocation.RIGHT_OUTPOST_CORNER));
        for (var calibration : calibrations) {
            operatorController
                    .start()
                    .and(calibration.getFirst())
                    .whileTrue(Commands.defer(
                            () -> Commands.waitTime(Seconds.of(0.1))
                                    .andThen(swerveSubsystem
                                            .resetPoseFromCalibrationPosition(calibration.getSecond())
                                            .andThen(
                                                    (autoManager != null && fuelSubsystem != null)
                                                            ? Commands.waitTime(Seconds.of(0.1))
                                                                    .andThen(autoManager.moveFromHubAndShoot())
                                                            : Commands.none())),
                            fuelSubsystem != null ? Set.of(swerveSubsystem, fuelSubsystem) : Set.of(swerveSubsystem)));
        }

        if (fuelSubsystem != null) {
            operatorController
                    .rightBumper()
                    .and(operatorController.start().negate())
                    .whileTrue(fuelSubsystem.requestAsOperator(OperatorFuelRequest.WINDUP));
            operatorController
                    .rightTrigger()
                    .and(operatorController.start().negate())
                    .and(operatorController.leftTrigger().negate())
                    .whileTrue(fuelSubsystem.requestAsOperator(OperatorFuelRequest.LAUNCH_WINDUP));
            operatorController
                    .rightTrigger()
                    .and(operatorController.start().negate())
                    .and(operatorController.leftTrigger())
                    .whileTrue(fuelSubsystem.requestAsOperator(OperatorFuelRequest.LAUNCH_NO_WINDUP));
            operatorController.leftBumper().whileTrue(fuelSubsystem.requestAsOperator(OperatorFuelRequest.EJECT));
            operatorController.a().whileTrue(fuelSubsystem.requestAsOperator(OperatorFuelRequest.INTAKE));
            operatorController.b().whileTrue(fuelSubsystem.requestAsOperator(OperatorFuelRequest.UNJAM));
            operatorController.leftStick().toggleOnTrue(shooterCalculator.temporarilyEnableManualMode());
            new Trigger(() -> operatorController.getLeftX() < -0.5)
                    .whileTrue(fuelSubsystem.decreaseManualLaunchVelocity());
            new Trigger(() -> operatorController.getLeftX() > 0.5)
                    .whileTrue(fuelSubsystem.increaseManualLaunchVelocity());
            new Trigger(() -> operatorController.getRightX() < -0.5)
                    .whileTrue(shooterCalculator.decreaseVelocityOffset());
            new Trigger(() -> operatorController.getRightX() > 0.5)
                    .whileTrue(shooterCalculator.increaseVelocityOffset());
            operatorController.back().whileTrue(fuelSubsystem.temporarilyUseMaxPower());
        }
        if (climberSubsystem != null) {
            operatorController
                    .y()
                    .whileTrue(climberSubsystem.climbCommand(
                            operatorController.start().negate(), operatorController.back()));
            operatorController
                    .x()
                    .whileTrue(climberSubsystem.armCommand(
                            operatorController.start().negate(), operatorController.back()));
            new LoggedNetworkButton("Climber/Set Climber Encoder to Vertical")
                    .getTrigger()
                    .onTrue(climberSubsystem.setClimberEncoderToVertical());
        }
        /*if (autoManager != null) {
            operatorController.back().whileTrue(autoManager.testAuto());
        }*/
    }

    public void initSmartDashboard() {
        SmartDashboard.putData("Auto/Start Location Chooser", autoStartLocationChooser);
        SmartDashboard.putBoolean("Auto/Auto Climb", autoClimb);
        SmartDashboard.putBoolean("Auto/Auto Outpost", autoOutpost);
        SmartDashboard.putBoolean("Auto/Auto Depot", autoDepot);
        SmartDashboard.putData("Power Distribution", powerDistribution);
        SmartDashboard.putData(
                "RobotContainer",
                builder -> builder.addBooleanProperty("Use Odometry", () -> useOdometry, v -> useOdometry = v));
        SmartDashboard.putData("Motor Info", motorInfo);
        SmartDashboard.putData("Live Field", field);
    }

    private void updateSmartDashboard() {
        autoClimb = SmartDashboard.getBoolean("Auto/Auto Climb", autoClimb);
        autoDepot = SmartDashboard.getBoolean("Auto/Auto Depot", autoDepot);
        autoOutpost = SmartDashboard.getBoolean("Auto/Auto Outpost", autoOutpost);
    }

    private void updateField() {
        field.setRobotPose(swerveSubsystem.getRobotPose());
        if (autoManager == null) return;
        final FieldObject2d trajectoryObject = field.getObject("BLine trajectory");
        final List<Pose2d> currentTrajectory = autoManager.getCurrentPoses();
        if (currentTrajectory == null) {
            trajectoryObject.setPoses();
            return;
        }
        trajectoryObject.setPoses(currentTrajectory);
    }

    public void preSchedulerUpdate() {
        shooterCalculator.prePeriodic();
        LoggedNetworkInput.runAllPeriodic();
    }

    public void postSchedulerUpdate() {
        NetworkTableInstance.getDefault().flush();
    }

    public void autonomousInit() {
        if (autoManager != null) {
            autoManager.autonomousInit();
        }
    }

    public void teleopInit() {
        if (autoManager != null) {
            autoManager.teleopInit();
        }
    }

    public void simulationInit() {
        shooterCalculator.simulationInit();
    }

    public void simulationPeriodic() {
        shooterCalculator.simulationPeriodic();
    }
}
