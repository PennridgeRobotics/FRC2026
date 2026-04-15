package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
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
import frc.robot.util.UnjamManager;
import frc.robot.util.controller.CommandJoystickController;
import frc.robot.util.dashboard.*;
import frc.robot.util.enums.Constants.ClimberConstants;
import frc.robot.util.enums.Constants.ControllerConstants;
import frc.robot.util.enums.Constants.FuelConstants;
import frc.robot.util.enums.Constants.LightConstants;
import frc.robot.util.enums.Constants.MiscConstants;
import frc.robot.util.enums.SpeedMultiplier;
import frc.robot.vision.VisionManager;
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
    private final VisionManager visionManager;
    private final MultiMotorInfoSendable motorInfo = new MultiMotorInfoSendable();
    private final @Nullable AutoManager autoManager;
    private final @Nullable UnjamManager unjamManager;

    private final StructPublisher<Pose2d> aheadRobotPose;
    private final StructPublisher<Pose2d> behindRobotPose;

    // Initializes controllers
    private final CommandXboxController driverController =
            new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController =
            new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);
    private final @Nullable CommandJoystickController joystickController = ControllerConstants.USING_JOYSTICK
            ? new CommandJoystickController(ControllerConstants.JOYSTICK_CONTROLLER_PORT)
            : null;

    private final SendableChooser<AutoManager.AutoStartLocation> autoStartLocationChooser;
    private final LoggedNetworkDouble autoStartDelaySecs = new LoggedNetworkDouble("/Auto/Start Delay", 0.0);
    private final LoggedNetworkBoolean autoShootAtStart1 = new LoggedNetworkBoolean("/Auto/1. Shoot at Start", true);
    private final LoggedNetworkBoolean autoCollectFromMidFast2 =
            new LoggedNetworkBoolean("/Auto/2. Collect From Mid (Fast)", false);
    private final LoggedNetworkBoolean autoCollectFromMidSlow3 =
            new LoggedNetworkBoolean("/Auto/3. Collect From Mid (Slow)", true);
    private final LoggedNetworkBoolean autoCollectFromMidSlow4 =
            new LoggedNetworkBoolean("/Auto/4. Collect From Mid (Slow)", true);
    private final LoggedNetworkBoolean autoDepot5 = new LoggedNetworkBoolean("/Auto/5. Auto Depot", false);
    private final LoggedNetworkBoolean autoOutpost6 = new LoggedNetworkBoolean("/Auto/6. Auto Outpost", false);
    private final LoggedNetworkBoolean autoCollectFromMidSlow7 =
            new LoggedNetworkBoolean("/Auto/7. Collect From Mid (Slow)", true);
    private final LoggedNetworkBoolean autoClimb8 = new LoggedNetworkBoolean("/Auto/8. Auto Climb", false);

    private final LoggedNetworkBoolean invertDriveControls =
            new LoggedNetworkBoolean("/Misc/Invert Drive Controls", false);
    private final LoggedNetworkBoolean useOdometry = new LoggedNetworkBoolean("/Misc/Use Odometry", true);
    private final Trigger useOdometryTrigger = new Trigger(useOdometry);
    private final Field2d field2d = new Field2d();
    private final LoggedNetworkSendable<Field2d> loggedField = new LoggedNetworkSendable<>("/Misc/Field", field2d);

    private final LoggedNetworkStructArray<Pose2d> loggedBLineTrajectory =
            new LoggedNetworkStructArray<>("/Misc/BLine Trajectory", Pose2d.struct, new Pose2d[0]);
    private final LoggedNetworkDouble loggedRumbleMaxValue = new LoggedNetworkDouble("Misc/Rumble Max Value", 0.5);
    private RumbleType rumbleType = RumbleType.kBothRumble;
    private double operatorRumbleValue = 0.0;
    private @Nullable Command operatorRumbleCommand;

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
        visionManager = swerveSubsystem.getVisionManager();
        shooterCalculator = swerveSubsystem.getShooterCalculator();
        fuelSubsystem = FuelConstants.FUEL_SUBSYSTEM_ENABLED ? new FuelSubsystem(shooterCalculator, motorInfo) : null;
        climberSubsystem = ClimberConstants.CLIMBER_ENABLED ? new ClimberSubsystem(motorInfo) : null;
        lightsSubsystem = LightConstants.LIGHTS_ENABLED
                ? new LightsSubsystem(swerveSubsystem, fuelSubsystem, climberSubsystem)
                : null;

        if (fuelSubsystem != null) {
            swerveSubsystem.setIsShootingTrigger(
                    fuelSubsystem.isWindingUpTrigger().or(fuelSubsystem.isLaunchingTrigger()));
        }

        // autoChooser = AutoBuilder.buildAutoChooser("Epic Auto");
        autoStartLocationChooser = new SendableChooser<>();
        for (final var location : AutoManager.AutoStartLocation.values()) {
            autoStartLocationChooser.addOption(StringUtils.capitalizeFully(location.name()), location);
        }

        autoManager = (fuelSubsystem != null)
                ? new AutoManager(swerveSubsystem, swerveSubsystem.getPathBuilder(), fuelSubsystem, climberSubsystem)
                : null;
        unjamManager = fuelSubsystem != null ? fuelSubsystem.getUnjamManager() : null;

        aheadRobotPose = NetworkTableInstance.getDefault()
                .getStructTopic("Robot Pose Ahead", Pose2d.struct)
                .publish();
        behindRobotPose = NetworkTableInstance.getDefault()
                .getStructTopic("Robot Pose Behind", Pose2d.struct)
                .publish();
        new LoggedNetworkSendable<>(
                "Misc/Rumble Type",
                SplitButtonChooser.withEnum(
                        () -> rumbleType,
                        Set.of(v -> {
                            operatorController.setRumble(rumbleType, 0.0);
                            rumbleType = v;
                        }),
                        rumbleType,
                        RumbleType.class));

        configureBindings();

        initSmartDashboard();

        HubTracker.isActive(); // initialize HubTracker
    }

    public @Nullable Command getAutonomousCommand() {
        return autoManager != null
                ? autoManager.getAutoCommand(new AutoManager.AutoOptions(
                        autoStartLocationChooser.getSelected(),
                        autoStartDelaySecs.getAsDouble(),
                        autoShootAtStart1.getAsBoolean(),
                        autoCollectFromMidFast2.getAsBoolean(),
                        autoCollectFromMidSlow3.getAsBoolean(),
                        autoCollectFromMidSlow4.getAsBoolean(),
                        autoDepot5.getAsBoolean(),
                        autoOutpost6.getAsBoolean(),
                        autoCollectFromMidSlow7.getAsBoolean(),
                        autoClimb8.getAsBoolean()))
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

        if (joystickController != null) {
            swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOrientedCommand(
                    () -> -joystickController.getY(),
                    () -> -joystickController.getX(),
                    () -> -joystickController.getTwist()));
            if (fuelSubsystem != null) {
                joystickController
                        .trigger(false, false)
                        .whileTrue(fuelSubsystem.launchCommand(true))
                        .whileTrue(swerveSubsystem.faceTowardsHubCommand());
                joystickController
                        .trigger(false, true)
                        .whileTrue(fuelSubsystem.windUpCommand())
                        .whileTrue(swerveSubsystem.faceTowardsHubCommand());
                joystickController.topHat(false, false).whileTrue(fuelSubsystem.intakeCommand());
            }
            if (autoManager != null) {
                joystickController.a1().whileTrue(autoManager.testAuto());
                joystickController.a2().whileTrue(autoManager.testOnePointPath());

                joystickController.b1().whileTrue(swerveSubsystem.lockPoseCommand());
            }
            return;
        }

        // for testing
        final var fieldOriented = true;
        final var forceRobotOrientedRotation = true;
        if (fieldOriented) {
            if (forceRobotOrientedRotation) {
                swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldAndRobotOrientedCommand(
                        () -> (invertDriveControls.getAsBoolean() ? 1 : -1) * driverController.getLeftY(),
                        () -> (invertDriveControls.getAsBoolean() ? 1 : -1) * driverController.getLeftX(),
                        () -> (invertDriveControls.getAsBoolean() ? 1 : -1) * driverController.getRightX(),
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
        driverController
                .start()
                .multiPress(3, 1.0)
                .onTrue(Commands.runOnce(() -> invertDriveControls.set(!invertDriveControls.getAsBoolean())));
        driverController.y().whileTrue(swerveSubsystem.faceTowardsHubCommand());
        driverController.x().whileTrue(swerveSubsystem.lockPoseCommand());
        if (fuelSubsystem != null && unjamManager != null) {
            driverController
                    .b()
                    .and(operatorController.leftTrigger().negate()) // let operator override
                    .and(unjamManager.isUsingSmartUnjamTrigger().negate()) // let smart unjam override
                    .whileTrue(fuelSubsystem.launchCommand(true))
                    .whileTrue(swerveSubsystem.faceTowardsHubCommand());
            driverController
                    .leftTrigger()
                    .and(operatorController.leftTrigger().negate()) // let operator override
                    .and(unjamManager.isUsingSmartUnjamTrigger().negate()) // let smart unjam override
                    .whileTrue(fuelSubsystem.intakeCommand());
        }
        new Trigger(() -> {
                    final var timeLeft = HubTracker.timeRemainingInCurrentShift();
                    return timeLeft != null && HubTracker.isActiveNext() && timeLeft.isEquivalent(Seconds.of(5));
                })
                .onTrue(Commands.sequence(
                        rumbleCommand(driverController, 0.3).withTimeout(Seconds.of(0.2)),
                        Commands.waitSeconds(0.2),
                        rumbleCommand(driverController, 0.3).withTimeout(Seconds.of(0.2))));
        /*driverController
        .a()
        .whileTrue(swerveSubsystem.resetPoseFromCalibrationPosition(
                PositionCalibrationLocation.FRONT_LEFT_OF_HUB));*/

        operatorController
                .rightBumper()
                .and(operatorController.start())
                .whileTrue(swerveSubsystem.straightenWheelsCommand());
        /*operatorController
                .leftTrigger()
                .and(operatorController.start())
                .onTrue(swerveSubsystem.toggleUseBackCameraInPoseEstimation());
        operatorController
                .rightTrigger()
                .and(operatorController.start())
                .onTrue(swerveSubsystem.toggleUseFrontCameraInPoseEstimation());*/

        operatorController.rightStick().onTrue(swerveSubsystem.toggleForceNormalDriveMode());

        if (autoManager != null) {
            operatorController.leftBumper().and(operatorController.start()).whileTrue(autoManager.testOnePointPath());
        }

        if (fuelSubsystem != null && unjamManager != null) {
            operatorController
                    .rightBumper()
                    .and(operatorController.start().negate())
                    .whileTrue(fuelSubsystem.requestAsOperator(OperatorFuelRequest.WINDUP));
            operatorController
                    .rightTrigger()
                    .and(operatorController.start().negate())
                    .and(operatorController.leftTrigger().negate())
                    .whileTrue(fuelSubsystem.requestAsOperator(OperatorFuelRequest.LAUNCH_WINDUP))
                    .whileTrue(swerveSubsystem.faceTowardsHubCommand());
            operatorController
                    .rightTrigger()
                    .and(operatorController.start().negate())
                    .and(operatorController.leftTrigger())
                    .whileTrue(fuelSubsystem.requestAsOperator(OperatorFuelRequest.LAUNCH_NO_WINDUP))
                    .whileTrue(swerveSubsystem.faceTowardsHubCommand());
            operatorController.leftBumper().whileTrue(fuelSubsystem.requestAsOperator(OperatorFuelRequest.EJECT));
            operatorController.a().whileTrue(fuelSubsystem.requestAsOperator(OperatorFuelRequest.INTAKE));
            operatorController.b().whileTrue(fuelSubsystem.requestAsOperator(OperatorFuelRequest.UNJAM));
            operatorController.leftStick().toggleOnTrue(shooterCalculator.temporarilyEnableManualMode());
            new Trigger(() -> operatorController.getLeftX() < -0.5)
                    .whileTrue(shooterCalculator.decreaseManualLaunchVelocity());
            new Trigger(() -> operatorController.getLeftX() > 0.5)
                    .whileTrue(shooterCalculator.increaseManualLaunchVelocity());
            new Trigger(() -> operatorController.getRightX() < -0.5)
                    .whileTrue(shooterCalculator.decreaseVelocityOffset());
            new Trigger(() -> operatorController.getRightX() > 0.5)
                    .whileTrue(shooterCalculator.increaseVelocityOffset());
            operatorController.back().whileTrue(unjamManager.temporarilyUseMaxPowerAll());
            operatorController
                    .leftTrigger()
                    .and(operatorController.start())
                    .whileTrue(unjamManager.smartUnjamCommand());
            unjamManager
                    .isUsingSmartUnjamTrigger()
                    .whileTrue(Commands.startEnd(() -> System.out.println("1"), () -> System.out.println("2")));
            unjamManager
                    .isUsingSmartUnjamTrigger()
                    .whileFalse(Commands.startEnd(() -> System.out.println("3"), () -> System.out.println("4")));
            new Trigger(() -> operatorRumbleValue > 0.01)
                    .whileTrue(Commands.runEnd(
                            () -> operatorController.setRumble(RumbleType.kRightRumble, operatorRumbleValue),
                            () -> operatorController.setRumble(RumbleType.kBothRumble, 0.0)));
            /*operatorController
            .start()
            .whileTrue(Commands.startEnd(
                    () -> operatorController.setRumble(rumbleType, loggedRumbleMaxValue.getAsDouble()),
                    () -> operatorController.setRumble(RumbleType.kBothRumble, 0.0)));*/
        }
        if (climberSubsystem != null) {
            operatorController
                    .y()
                    .whileTrue(climberSubsystem.climbCommand(
                            operatorController.start().negate(), operatorController.back()))
                    .whileTrue(swerveSubsystem.straightenWheelsCommand());
            operatorController
                    .x()
                    .whileTrue(climberSubsystem.armCommand(
                            operatorController.start().negate(), operatorController.back()))
                    .whileTrue(swerveSubsystem.straightenWheelsCommand());
        }
        /*if (autoManager != null) {
            operatorController.back().whileTrue(autoManager.testAuto());
        }*/
    }

    public void initSmartDashboard() {
        new LoggedNetworkSendable<>("/Auto/Start Location Chooser", autoStartLocationChooser);
        new LoggedNetworkSendable<>("/Misc/Power Distribution", powerDistribution);
        new LoggedNetworkSendable<>("/Misc/Motor Info", motorInfo);
        // new LoggedNetworkSendable<>("/Pigeon2", new Pigeon2Sendable(swerveSubsystem.getPigeon2()));
        /*final var emptyPoseArray = new Pose2d[0];
        new LoggedNetworkStructArray<>("/Misc/BLine Completed Poses", Pose2d.struct, () -> {
            if (autoManager == null) return emptyPoseArray;
            final var completedPoses = autoManager.getCompletedPoses();
            if (completedPoses == null) return emptyPoseArray;
            return completedPoses.toArray(new Pose2d[0]);
        });
        new LoggedNetworkStructArray<>("/Misc/BLine Poses to Complete", Pose2d.struct, () -> {
            if (autoManager == null) return emptyPoseArray;
            final var posesToComplete = autoManager.getPosesToComplete();
            if (posesToComplete == null) return emptyPoseArray;
            return posesToComplete.toArray(new Pose2d[0]);
        });*/
    }

    private Command rumbleCommand(CommandXboxController controller, double strength) {
        return Commands.runEnd(
                () -> controller.setRumble(RumbleType.kRightRumble, strength),
                () -> controller.setRumble(RumbleType.kBothRumble, 0.0));
    }

    private void updateField() {
        if (autoManager == null) return;
        field2d.setRobotPose(swerveSubsystem.getRobotPose());
        final FieldObject2d trajectoryObject = field2d.getObject("BLine trajectory");
        final List<Pose2d> currentTrajectory = autoManager.getCurrentPoses();
        if (currentTrajectory == null) {
            trajectoryObject.setPoses();
            loggedBLineTrajectory.set(new Pose2d[0]);
            return;
        }
        trajectoryObject.setPoses(currentTrajectory);
        loggedBLineTrajectory.set(currentTrajectory.toArray(Pose2d[]::new));
    }

    public void preSchedulerUpdate() {
        shooterCalculator.prePeriodic();
        LoggedNetworkInput.runAllPeriodic();
    }

    public void postSchedulerUpdate() {
        // NetworkTableInstance.getDefault().flush();
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
        setupOperatorCommand();
    }

    private void setupOperatorCommand() {
        if (operatorRumbleCommand != null) CommandScheduler.getInstance().cancel(operatorRumbleCommand);
        if (unjamManager == null) return;
        operatorRumbleCommand = Commands.run(() -> {
            final var intakeLauncherStalled =
                    unjamManager.intakeLauncherStalledTrigger().getAsBoolean();
            final var indexerStalled = unjamManager.indexerStalledTrigger().getAsBoolean();
            final var totalStalled = intakeLauncherStalled ? (indexerStalled ? 2 : 1) : (indexerStalled ? 1 : 0);
            final boolean usingSmartUnjam =
                    unjamManager.isUsingSmartUnjamTrigger().getAsBoolean();
            final boolean isReady = unjamManager.isReadyTrigger().getAsBoolean();
            operatorRumbleValue = usingSmartUnjam
                    ? (isReady ? 0.3 : 0)
                    : switch (totalStalled) {
                        case 0 -> 0;
                        case 1 -> 0.2;
                        default -> 1.0;
                    };
        });
        CommandScheduler.getInstance().schedule(operatorRumbleCommand);
    }

    public void simulationInit() {
        shooterCalculator.simulationInit();
    }

    public void simulationPeriodic() {
        shooterCalculator.simulationPeriodic();
    }

    public void disabledInit() {
        operatorController.setRumble(RumbleType.kBothRumble, 0.0);
    }
}
