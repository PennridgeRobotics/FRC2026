package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.core.CorePigeon2;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.util.BumpManager;
import frc.robot.util.ShooterCalculator;
import frc.robot.util.SlewRateLimiter2d;
import frc.robot.util.dashboard.LoggedNetworkBoolean;
import frc.robot.util.dashboard.LoggedNetworkDouble;
import frc.robot.util.dashboard.LoggedNetworkSendable;
import frc.robot.util.dashboard.LoggedNetworkStruct;
import frc.robot.util.dashboard.LoggedNetworkUnit;
import frc.robot.util.dashboard.MultiMotorInfoSendable;
import frc.robot.util.dashboard.PIDSendable;
import frc.robot.util.dashboard.PIDSendable.PIDValues;
import frc.robot.util.dashboard.SplitButtonChooser;
import frc.robot.util.enums.Constants.ControllerConstants;
import frc.robot.util.enums.Constants.DriveConstants;
import frc.robot.util.enums.Constants.FieldConstants;
import frc.robot.util.enums.Constants.PhysicalConstants;
import frc.robot.util.enums.Constants.ShootOnTheMoveConstants;
import frc.robot.util.enums.Constants.VisionConstants;
import frc.robot.util.enums.PositionCalibrationLocation;
import frc.robot.util.enums.SpeedMultiplier;
import frc.robot.vision.PhotonCamera;
import frc.robot.vision.VisionManager;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

@NullMarked
public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private VisionManager visionManager;
    private final BumpManager bumpManager;
    private final ShooterCalculator shooterCalculator;
    private final MultiMotorInfoSendable motorInfo;

    private boolean forceNormalDriveMode = false;
    private boolean lockYawTowardsVelocity = false;
    private boolean faceTowardsHub = false;
    private SpeedMultiplier speedMultiplier = SpeedMultiplier.NORMAL;
    private final BooleanSupplier headingCorrectionSupplier;
    private final DoubleSupplier headingCorrectionDeadband;

    private final Trigger forceNormalDriveModeTrigger = new Trigger(() -> forceNormalDriveMode);

    private final PIDController bLineTranslationPID =
            Robot.isReal() ? new PIDController(5.0, 0, 1.3) : new PIDController(1.9, 0.1, 0.4);
    private final PIDController bLineRotationPID =
            Robot.isReal() ? new PIDController(5.0, 0, 0.85) : new PIDController(5.0, 0.2, 0.6);
    private final PIDController bLineCrossTrackPID = new PIDController(2.0, 0, 0);
    private final FollowPath.Builder pathBuilder;

    private final SlewRateLimiter2d linearDriveLimiter =
            new SlewRateLimiter2d(DriveConstants.MAX_LINEAR_ACCELERATION.in(MetersPerSecondPerSecond));

    private final LoggedNetworkUnit<LinearVelocityUnit, LinearVelocity> loggedMaxVelocityWhileShooting;
    private final LoggedNetworkStruct<Pose2d> loggedRobotPose;
    private final LoggedNetworkUnit<LinearVelocityUnit, LinearVelocity> loggedTargetLinearVelocity;
    private final LoggedNetworkStruct<Translation2d> loggedTargetTranslation;
    private final LoggedNetworkUnit<AngularVelocityUnit, AngularVelocity> loggedTargetAngularVelocity;
    private final LoggedNetworkBoolean loggedUsingSOTMHubLock;
    private SOTMHubLockType sotmHubLockType = SOTMHubLockType.ANGLE_LOCK_AND_VELOCITY_FF;

    @SuppressWarnings("StaticAssignmentInConstructor")
    public SwerveSubsystem(final MultiMotorInfoSendable motorInfo) throws IOException {
        this.motorInfo = motorInfo;
        headingCorrectionSupplier = new LoggedNetworkBoolean("Swerve/Heading Correction Enabled", false);
        headingCorrectionDeadband = new LoggedNetworkDouble("Swerve/Heading Correction Deadband", 0.01);
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        swerveDrive = new SwerveParser(
                        new File(Filesystem.getDeployDirectory(), DriveConstants.SWERVE_CONFIG_DIRECTORY))
                .createSwerveDrive(
                        DriveConstants.MAX_LINEAR_SPEED.in(MetersPerSecond),
                        new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)), Rotation2d.kZero));
        swerveDrive.setHeadingCorrection(
                headingCorrectionSupplier.getAsBoolean()); // enable this after testing/tuning PID
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // disable for simulations
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1); // may need to adjust; see docs
        // swerveDrive.useExternalFeedbackSensor();
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1); // can set to true, but I want to test
        // swerveDrive.setMotorIdleMode(true);

        bumpManager = new BumpManager(
                getPigeon2(), swerveDrive::getGyroRotation3d, this::getRobotPose, forceNormalDriveModeTrigger);
        shooterCalculator = new ShooterCalculator(swerveDrive);

        loggedMaxVelocityWhileShooting = new LoggedNetworkUnit<>(
                "Shooter Calculator/Max Velocity While Shooting", ShootOnTheMoveConstants.MAX_VELOCITY_WHILE_SHOOTING);
        loggedRobotPose = new LoggedNetworkStruct<>("Robot Pose Struct", Pose2d.struct, swerveDrive.getPose());
        loggedRobotPose.addListener(this::resetPose);
        loggedTargetLinearVelocity = new LoggedNetworkUnit<>("/Swerve/Target Linear Velocity", MetersPerSecond.zero());
        loggedTargetTranslation =
                new LoggedNetworkStruct<>("/Swerve/Target Translation", Translation2d.struct, new Translation2d());
        loggedTargetAngularVelocity =
                new LoggedNetworkUnit<>("/Swerve/Target Angular Velocity", DegreesPerSecond.zero());
        loggedUsingSOTMHubLock = new LoggedNetworkBoolean("/Swerve/Using SOTM Hub Lock", false);
        new LoggedNetworkSendable<>(
                "Swerve/SOTM Hub Lock Type",
                new SplitButtonChooser<>(
                        () -> sotmHubLockType,
                        Arrays.asList(SOTMHubLockType.values()),
                        Set.of((newType) -> sotmHubLockType = newType),
                        sotmHubLockType,
                        SOTMHubLockType::fromDashboardName,
                        SOTMHubLockType::getDashboardName));

        setupVisionManager();
        pathBuilder = setupBLine();
        initSmartDashboard();
    }

    private void initSmartDashboard() {
        final var moduleNames = List.of("Front-Left", "Front-Right", "Back-Right", "Back-Left");
        for (int i = 0; i < swerveDrive.getModules().length; i++) {
            final var swerveModule = swerveDrive.getModules()[i];
            motorInfo.addMotor((SparkMax) swerveModule.getDriveMotor().getMotor(), moduleNames.get(i) + " Drive");
            motorInfo.addMotor((SparkMax) swerveModule.getAngleMotor().getMotor(), moduleNames.get(i) + " Angle");
        }
        SmartDashboard.putData("Swerve Subsystem", builder -> {
            builder.addStringProperty(
                    "Bump Status",
                    () -> {
                        if (!bumpManager.isRawBumpLockEnabledTrigger().getAsBoolean()) return Color.kLime.toHexString();
                        if (bumpManager.isOnBumpTrigger().getAsBoolean()) return Color.kRed.toHexString();
                        return Color.kYellow.toHexString(); // in bump area, but not on the bump itself
                    },
                    null);
            builder.addBooleanProperty(
                    "Driver Overrides/Force Normal Drive Mode",
                    () -> forceNormalDriveMode,
                    v -> forceNormalDriveMode = v);
            builder.addBooleanProperty(
                    "Driver Overrides/Lock Yaw Towards Velocity",
                    () -> lockYawTowardsVelocity,
                    v -> lockYawTowardsVelocity = v);
            builder.addBooleanProperty(
                    "Driver Overrides/Face Towards Hub", () -> faceTowardsHub, v -> faceTowardsHub = v);
        });
        SmartDashboard.putData("Robot Pose", builder -> {
            builder.addDoubleProperty("Pose X (m)", () -> getRobotPose().getX(), newX -> {
                final var pose = getRobotPose();
                resetPose(new Pose2d(newX, pose.getY(), pose.getRotation()));
            });
            builder.addDoubleProperty("Pose Y (m)", () -> getRobotPose().getY(), newY -> {
                final var pose = getRobotPose();
                resetPose(new Pose2d(pose.getX(), newY, pose.getRotation()));
            });
            builder.addDoubleProperty(
                    "Pose Rotation (deg)", () -> getRobotPose().getRotation().getDegrees(), newRotation -> {
                        final var pose = getRobotPose();
                        resetPose(new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(newRotation)));
                    });
        });
        SmartDashboard.putData(
                "Swerve Controller Heading PID",
                new PIDSendable(
                        swerveDrive.swerveController.thetaController,
                        PIDSendable.Type.PID,
                        PIDValues.from(swerveDrive.swerveController.thetaController)));
    }

    @Override
    public void periodic() {
        if (headingCorrectionSupplier.getAsBoolean() != swerveDrive.headingCorrection) {
            swerveDrive.setHeadingCorrection(
                    headingCorrectionSupplier.getAsBoolean(), headingCorrectionDeadband.getAsDouble());
        }
        loggedRobotPose.set(getRobotPose());
    }

    private void updateOdometry() {
        swerveDrive.updateOdometry();
        visionManager.updatePoseEstimation(swerveDrive);
    }

    /**
     * @param xVelocity Positive = towards the other alliance
     * @param yVelocity Positive = towards the left wall
     * @param angularVelocity Positive = CCW
     */
    public Command driveFieldOrientedCommand(
            final Supplier<LinearVelocity> xVelocity,
            final Supplier<LinearVelocity> yVelocity,
            final Supplier<AngularVelocity> angularVelocity) {
        return run(() -> driveFieldOriented(xVelocity.get(), yVelocity.get(), angularVelocity.get()));
    }

    /**
     * @param xInput [-1,1] Positive = towards the other alliance
     * @param yInput [-1,1] Positive = towards the left wall
     * @param angularInput [-1,1] Positive = CCW
     */
    public Command driveFieldOrientedCommand(
            final DoubleSupplier xInput, final DoubleSupplier yInput, final DoubleSupplier angularInput) {
        return driveFieldOrientedCommand(
                () -> joystickToLinearVelocity(xInput.getAsDouble()),
                () -> joystickToLinearVelocity(yInput.getAsDouble()),
                () -> joystickToAngularVelocity(angularInput.getAsDouble()));
    }

    /**
     * @param xVelocity Positive = towards the other alliance
     * @param yVelocity Positive = towards the left wall
     * @param headingX [-1,1] Heading X (positive = front)
     * @param headingY [-1,1] Heading Y (positive = left)
     */
    public Command driveFieldOrientedHeadingCommand(
            final Supplier<LinearVelocity> xVelocity,
            final Supplier<LinearVelocity> yVelocity,
            final DoubleSupplier headingX,
            final DoubleSupplier headingY) {
        return run(() ->
                driveFieldOriented(xVelocity.get(), yVelocity.get(), headingX.getAsDouble(), headingY.getAsDouble()));
    }

    /**
     * @param xInput [-1,1] Positive = towards the other alliance
     * @param yInput [-1,1] Positive = towards the left wall
     * @param headingX [-1,1] Heading X (positive = front)
     * @param headingY [-1,1] Heading Y (positive = left)
     */
    public Command driveFieldOrientedHeadingCommand(
            final DoubleSupplier xInput,
            final DoubleSupplier yInput,
            final DoubleSupplier headingX,
            final DoubleSupplier headingY) {
        return driveFieldOrientedHeadingCommand(
                () -> joystickToLinearVelocity(xInput.getAsDouble()),
                () -> joystickToLinearVelocity(yInput.getAsDouble()),
                headingX,
                headingY);
    }
    /**
     * @param xVelocity Positive = towards the other alliance
     * @param yVelocity Positive = towards the left wall
     * @param heading Heading to point the robot in
     */
    public Command driveFieldOrientedHeadingCommand(
            final Supplier<LinearVelocity> xVelocity,
            final Supplier<LinearVelocity> yVelocity,
            final Supplier<Rotation2d> heading) {
        return run(() -> driveFieldOriented(xVelocity.get(), yVelocity.get(), getTargetAngularVelocity(heading.get())));
    }

    /**
     * Command to drive the robot using translative values and heading as angular velocity.
     *
     * @param translationX Translation in the X direction.
     * @param translationY Translation in the Y direction.
     * @param angularRotationX Rotation of the robot to set
     * @return Drive command.
     */
    public Command driveRobotOrientedCommand(
            DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return run(() -> {
            // Make the robot move
            final var chassisSpeeds = new ChassisSpeeds(
                    translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                    translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                    angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity());

            final var fieldOriented = ChassisSpeeds.fromRobotRelativeSpeeds(
                    chassisSpeeds, getRobotPose().getRotation());
            driveFieldOriented(
                    MetersPerSecond.of(fieldOriented.vxMetersPerSecond),
                    MetersPerSecond.of(fieldOriented.vyMetersPerSecond),
                    RadiansPerSecond.of(fieldOriented.omegaRadiansPerSecond));

            /*if (lockYawTowardsVelocity && !forceNormalDriveMode) {
                chassisSpeeds.omegaRadiansPerSecond = getVelocityAngle(
                                MetersPerSecond.of(chassisSpeeds.vxMetersPerSecond),
                                MetersPerSecond.of(chassisSpeeds.vyMetersPerSecond))
                        .getRadians();
            }
            swerveDrive.drive(chassisSpeeds, false, new Translation2d());*/
        });
    }

    /**
     * @param xInput [-1,1] Positive = towards the other alliance
     * @param yInput [-1,1] Positive = towards the left wall
     * @param angularInput [-1,1] Positive = CCW
     * @param headingX [-1,1] Heading X (positive = front) - OVERRIDES {@code angularInput}
     * @param headingY [-1,1] Heading Y (positive = left) - OVERRIDES {@code angularInput}
     */
    public Command driveFieldAndRobotOrientedCommand(
            final DoubleSupplier xInput,
            final DoubleSupplier yInput,
            final DoubleSupplier angularInput,
            final DoubleSupplier headingX,
            final DoubleSupplier headingY) {
        return run(() -> {
            final var xVelocity = joystickToLinearVelocity(xInput.getAsDouble());
            final var yVelocity = joystickToLinearVelocity(yInput.getAsDouble());
            final var headingXValue = headingX.getAsDouble();
            final var headingYValue = headingY.getAsDouble();
            if (!swerveDrive.swerveController.withinHypotDeadband(headingXValue, headingYValue)) {
                driveFieldOriented(xVelocity, yVelocity, headingXValue, headingYValue);
                return;
            }
            final var angularVelocity = joystickToAngularVelocity(angularInput.getAsDouble());
            driveFieldOriented(xVelocity, yVelocity, angularVelocity);
        });
    }

    private void driveFieldOriented(
            final LinearVelocity xVelocity,
            final LinearVelocity yVelocity,
            final double headingX,
            final double headingY) {
        driveFieldOriented(xVelocity, yVelocity, getTargetAngularVelocity(calculateTargetAngle(headingX, headingY)));
    }

    private void driveFieldOriented(
            final LinearVelocity xVelocity, final LinearVelocity yVelocity, final AngularVelocity angularVelocity) {
        final var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return;
        }
        var linearVelocity = new Translation2d(xVelocity.in(MetersPerSecond), yVelocity.in(MetersPerSecond));
        if (!forceNormalDriveMode
                && faceTowardsHub
                && getShooterCalculator().isUsingSOTM()
                && linearVelocity.getNorm()
                        > loggedMaxVelocityWhileShooting.get().in(MetersPerSecond)) {
            linearVelocity = linearVelocity
                    .div(linearVelocity.getNorm())
                    .times(loggedMaxVelocityWhileShooting.get().in(MetersPerSecond));
        }
        if (DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red) { // flip if red
            linearVelocity = linearVelocity.unaryMinus();
        }
        final Translation2d limitedLinearVelocity = linearDriveLimiter.calculate(linearVelocity);
        final AngularVelocity determinedAngularVelocity;
        boolean usingSOTMHubLock = false;
        if (forceNormalDriveMode) {
            determinedAngularVelocity = angularVelocity;
        } else if (faceTowardsHub) {
            if (getShooterCalculator().isUsingSOTM()) {
                determinedAngularVelocity = switch (sotmHubLockType) {
                    case ANGLE_LOCK -> getTargetAngularVelocity(getAngleToHub());
                    case VELOCITY_FF ->
                        getShooterCalculator()
                                        .calculateShotData()
                                        .driveAngleFF()
                                        .isEquivalent(DegreesPerSecond.zero())
                                ? getTargetAngularVelocity(getAngleToHub())
                                : getShooterCalculator().calculateShotData().driveAngleFF();
                    case ANGLE_LOCK_AND_VELOCITY_FF ->
                        getShooterCalculator()
                                .calculateShotData()
                                .driveAngleFF()
                                .plus(getTargetAngularVelocity(getAngleToHub()));
                };
                usingSOTMHubLock = true;
            } else determinedAngularVelocity = getTargetAngularVelocity(getAngleToHub());
        } else if (lockYawTowardsVelocity) {
            determinedAngularVelocity = getTargetAngularVelocity(getVelocityAngle(
                    MetersPerSecond.of(limitedLinearVelocity.getX()),
                    MetersPerSecond.of(limitedLinearVelocity.getY())));
        } else if (bumpManager.isBumpLockEnabledTrigger().getAsBoolean()) {
            determinedAngularVelocity = getTargetAngularVelocity(bumpManager.getBumpLockAngle());
        } else {
            determinedAngularVelocity = angularVelocity;
        }
        loggedUsingSOTMHubLock.set(usingSOTMHubLock);
        final AngularVelocity maxAngularVelocity = RadiansPerSecond.of(swerveDrive.getMaximumChassisAngularVelocity());
        final AngularVelocity finalAngularVelocity = determinedAngularVelocity.gt(maxAngularVelocity)
                ? maxAngularVelocity
                : (determinedAngularVelocity.lt(maxAngularVelocity.unaryMinus())
                        ? maxAngularVelocity.unaryMinus()
                        : determinedAngularVelocity);

        loggedTargetLinearVelocity.set(MetersPerSecond.of(limitedLinearVelocity.getNorm()));
        loggedTargetTranslation.set(limitedLinearVelocity);
        loggedTargetAngularVelocity.set(finalAngularVelocity);

        shooterCalculator.setLastAngularVelocityInput(
                !forceNormalDriveMode && faceTowardsHub ? DegreesPerSecond.zero() : determinedAngularVelocity);
        swerveDrive.driveFieldOriented(new ChassisSpeeds(
                limitedLinearVelocity.getX(), limitedLinearVelocity.getY(), finalAngularVelocity.in(RadiansPerSecond)));
    }

    public void driveRobotOriented(final ChassisSpeeds chassisSpeeds) {
        final var fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, swerveDrive.getYaw());
        driveFieldOriented(
                MetersPerSecond.of(fieldRelative.vxMetersPerSecond),
                MetersPerSecond.of(fieldRelative.vyMetersPerSecond),
                RadiansPerSecond.of(fieldRelative.omegaRadiansPerSecond));
    }

    public Command driveFieldOrientedTestCommand(
            DoubleSupplier translationX,
            DoubleSupplier translationY,
            DoubleSupplier headingX,
            DoubleSupplier headingY) {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of
        // control.
        return run(() -> {
            Translation2d scaledInputs = SwerveMath.scaleTranslation(
                    new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

            swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
                    scaledInputs.getX(),
                    scaledInputs.getY(),
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumChassisVelocity()));
        });
    }

    public Command stopDrivingCommand() {
        return run(() -> driveFieldOriented(MetersPerSecond.zero(), MetersPerSecond.zero(), DegreesPerSecond.zero()))
                .until(() -> MathUtil.isNear(
                        0.0, linearDriveLimiter.getPrevTranslation().getSquaredNorm(), 0.0001));
    }

    public Command centerModulesCommand() {
        return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(mod -> mod.setAngle(0.0)));
    }

    public Command lockPoseCommand() {
        return run(swerveDrive::lockPose);
    }

    public Command sysIdDriveMotorCommand() {
        // Empty config defaults to 1 Volt/sec ramp rate and 7 Volt step voltage
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(new SysIdRoutine.Config(), this, swerveDrive, 12, true),
                3.0,
                5.0,
                3.0);
    }

    public Command sysIdAngleMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(new SysIdRoutine.Config(), this, swerveDrive), 3.0, 5.0, 3.0);
    }

    public Command forceNormalDriveMode(boolean force) {
        return runOnce(() -> forceNormalDriveMode = force);
    }

    public void zeroGyroWithAlliance() {
        swerveDrive.zeroGyro();
        if (Alliance.Red.equals(DriverStation.getAlliance().orElse(null))) {
            swerveDrive.resetOdometry(new Pose2d(getRobotPose().getTranslation(), Rotation2d.k180deg));
        }
    }

    public Trigger isInBumpZoneTrigger() {
        return bumpManager.isInBumpZoneTrigger();
    }

    public Trigger isOnBumpTrigger() {
        return bumpManager.isOnBumpTrigger();
    }

    public Trigger isBumpLockOverriddenTrigger() {
        return bumpManager.isBumpLockOverriddenTrigger();
    }

    public Command faceTowardsHubCommand() {
        return Commands.runEnd(() -> faceTowardsHub = true, () -> faceTowardsHub = false);
    }

    public Rotation2d getAngleToHub() {
        return getShooterCalculator().calculateShotData().heading();
    }

    public Command straightenWheelsCommand() {
        return Commands.run(() -> setModuleOrientations(Rotation2d.kZero));
    }

    public Command lockYawTowardsVelocity() {
        return Commands.startEnd(() -> lockYawTowardsVelocity = true, () -> lockYawTowardsVelocity = false);
    }

    public Command enableManualBumpLock() {
        return bumpManager.enableManualBumpLock();
    }

    public Rotation2d getBumpLockAngle() {
        return bumpManager.getBumpLockAngle();
    }

    public Command setSpeedMultiplierCommand(SpeedMultiplier speedMultiplier) {
        return Commands.runOnce(() -> this.speedMultiplier = speedMultiplier);
    }

    public Command resetPoseFromCalibrationPosition(PositionCalibrationLocation location) {
        return Commands.runOnce(() -> {
            final var currentRot = getRobotPose().getRotation().getDegrees();
            final var flip = DriverStation.getAlliance().orElse(null) == Alliance.Red;
            final var invertXY = MathUtil.isNear(90.0, Math.abs(currentRot) % 180, 45.0);
            var xPos = (invertXY ? PhysicalConstants.ROBOT_WIDTH_Y : PhysicalConstants.ROBOT_LENGTH_X).div(2);
            var yPos = (invertXY ? PhysicalConstants.ROBOT_LENGTH_X : PhysicalConstants.ROBOT_WIDTH_Y).div(2);

            switch (location) {
                case LEFT_DEPOT_CORNER -> yPos = FieldConstants.FIELD_WIDTH_Y.minus(yPos);
                case RIGHT_OUTPOST_CORNER -> {} // (0, 0) is the top-right corner
                case LEFT_TRENCH_INNER -> {
                    xPos = FieldConstants.TRENCH_X.minus(xPos);
                    yPos = FieldConstants.FIELD_WIDTH_Y
                            .minus(FieldConstants.TRENCH_TO_EDGE_Y)
                            .plus(yPos);
                }
                case RIGHT_TRENCH_INNER -> {
                    xPos = FieldConstants.TRENCH_X.minus(xPos);
                    yPos = FieldConstants.TRENCH_TO_EDGE_Y.minus(yPos);
                }
                case LEFT_TRENCH_OUTER -> {
                    xPos = FieldConstants.TRENCH_X.minus(xPos);
                    yPos = FieldConstants.FIELD_WIDTH_Y.minus(yPos);
                }
                case RIGHT_TRENCH_OUTER -> xPos = FieldConstants.TRENCH_X.minus(xPos);
                case FRONT_LEFT_OF_HUB -> {
                    xPos = FieldConstants.HUB_BLUE
                            .getMeasureX()
                            .minus(FieldConstants.HUB_WIDTH_X.div(2))
                            .minus(xPos);
                    yPos = FieldConstants.HUB_BLUE
                            .getMeasureY()
                            .plus(FieldConstants.HUB_LENGTH_Y.div(2))
                            .minus(yPos);
                }
                case FRONT_RIGHT_OF_HUB -> {
                    xPos = FieldConstants.HUB_BLUE
                            .getMeasureX()
                            .minus(FieldConstants.HUB_WIDTH_X.div(2))
                            .minus(xPos);
                    yPos = FieldConstants.HUB_BLUE
                            .getMeasureY()
                            .minus(FieldConstants.HUB_LENGTH_Y.div(2))
                            .plus(yPos);
                }
                case FRONT_CENTER_OF_HUB -> {
                    xPos = FieldConstants.HUB_BLUE
                            .getMeasureX()
                            .minus(FieldConstants.HUB_WIDTH_X.div(2))
                            .minus(xPos);
                    yPos = FieldConstants.HUB_BLUE.getMeasureY();
                }
            }

            if (flip) {
                xPos = FieldConstants.FIELD_LENGTH_X.minus(xPos);
                yPos = FieldConstants.FIELD_WIDTH_Y.minus(yPos);
            }
            final Pose2d newPose = new Pose2d(xPos, yPos, getRobotPose().getRotation());
            resetPose(newPose);
        });
    }

    private Rotation2d getVelocityAngle() {
        final var fieldVelocity = swerveDrive.getFieldVelocity();
        return getVelocityAngle(
                MetersPerSecond.of(fieldVelocity.vxMetersPerSecond),
                MetersPerSecond.of(fieldVelocity.vyMetersPerSecond));
    }

    private Rotation2d getVelocityAngle(LinearVelocity xVelocity, LinearVelocity yVelocity) {
        final double velocityAngle = Math.atan2(yVelocity.in(MetersPerSecond), xVelocity.in(MetersPerSecond));
        return Rotation2d.fromRadians(velocityAngle);
    }

    private AngularVelocity getTargetAngularVelocity(Rotation2d targetAngle) {
        final var currentHeading = swerveDrive.getOdometryHeading().getRadians();
        final var targetHeading = targetAngle.getRadians();
        final var maxAngularVelocity = getMaximumChassisAngularVelocity().in(RadiansPerSecond);
        final var calculated = swerveDrive.swerveController.thetaController.calculate(currentHeading, targetHeading)
                * maxAngularVelocity;
        final var limited = swerveDrive.swerveController.angleLimiter != null
                ? swerveDrive.swerveController.angleLimiter.calculate(calculated)
                : calculated;
        return RadiansPerSecond.of(limited);
    }

    private Rotation2d calculateTargetAngle(double headingX, double headingY) {
        return Rotation2d.fromRadians(swerveDrive.swerveController.getJoystickAngle(headingX, headingY));
    }

    private LinearVelocity joystickToLinearVelocity(final double input) {
        final var withDeadband =
                MathUtil.applyDeadband(input, ControllerConstants.DRIVE_MIN_INPUT, ControllerConstants.DRIVE_MAX_INPUT);
        final var scaled = Math.abs(Math.pow(withDeadband, ControllerConstants.LINEAR_DRIVE_POWER_SCALE))
                * Math.signum(withDeadband);
        return getMaximumChassisVelocity().times(scaled).times(speedMultiplier.getMultiplier());
    }

    private AngularVelocity joystickToAngularVelocity(final double input) {
        final var withDeadband =
                MathUtil.applyDeadband(input, ControllerConstants.DRIVE_MIN_INPUT, ControllerConstants.DRIVE_MAX_INPUT);
        final var scaled = Math.abs(Math.pow(withDeadband, ControllerConstants.ROTATE_DRIVE_POWER_SCALE))
                * Math.signum(withDeadband);
        return getMaximumChassisAngularVelocity().times(scaled).times(speedMultiplier.getMultiplier());
    }

    private void setModuleOrientations(Rotation2d rotation) {
        final var states = new SwerveModuleState[swerveDrive.getModules().length];
        Arrays.fill(states, new SwerveModuleState(0, rotation));
        swerveDrive.setModuleStates(states, false);
        for (int i = 0; i < swerveDrive.getModules().length; i++) {
            final var swerveModule = swerveDrive.getModules()[i];
            final var state = states[i];
            final var originalAngle = state.angle;
            swerveModule.applyAntiJitter(state, false);
            if (state.angle.equals(originalAngle)) {
                continue; // anti-jitter wasn't applied! (or rotation isn't necessary)
            }
            state.angle = originalAngle;
            swerveModule.setDesiredState(state, false, true); // force setting state to bypass anti-jitter
        }
    }

    private FollowPath.Builder setupBLine() {
        new LoggedNetworkSendable<>(
                "BLine/Translation PID", new PIDSendable(bLineTranslationPID, PIDSendable.Type.PID));
        new LoggedNetworkSendable<>("BLine/Rotation PID", new PIDSendable(bLineRotationPID, PIDSendable.Type.PID));
        new LoggedNetworkSendable<>("BLine/Cross Track PID", new PIDSendable(bLineCrossTrackPID, PIDSendable.Type.PID));
        return new FollowPath.Builder(
                        this,
                        this::getRobotPose,
                        swerveDrive::getRobotVelocity,
                        this::driveRobotOriented,
                        bLineTranslationPID,
                        bLineRotationPID,
                        bLineCrossTrackPID)
                .withDefaultShouldFlip()
                .withPoseReset(this::resetPose);
    }

    public void resetPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
        swerveDrive.swerveDrivePoseEstimator.resetRotation(pose.getRotation());
    }

    private SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    public Pose2d getRobotPose() {
        return swerveDrive.getPose();
    }

    private LinearVelocity getMaximumChassisVelocity() {
        return MetersPerSecond.of(swerveDrive.getMaximumChassisVelocity());
    }

    private AngularVelocity getMaximumChassisAngularVelocity() {
        return RadiansPerSecond.of(swerveDrive.getMaximumChassisAngularVelocity());
    }

    private void setupVisionManager() {
        this.visionManager = new VisionManager(() -> swerveDrive
                .getSimulationDriveTrainPose()
                .orElseThrow(
                        () -> new IllegalStateException("Cannot get simulation drive train pose when not simulating")));

        swerveDrive.stopOdometryThread();
        @SuppressWarnings("resource")
        final var odometryThread = new Notifier(this::updateOdometry);
        odometryThread.setName("Odometry Thread");
        odometryThread.startPeriodic(Robot.isSimulation() ? 0.01 : 0.02);

        if (!VisionConstants.VISION_ENABLED) {
            return;
        }
        visionManager.addCameras(
                new PhotonCamera(
                        VisionConstants.CAMERA_BACK_NAME,
                        VisionConstants.CAMERA_BACK_TRANSLATION,
                        VisionConstants.CAMERA_BACK_ROTATION),
                new PhotonCamera(
                        VisionConstants.CAMERA_FRONT_NAME,
                        VisionConstants.CAMERA_FRONT_TRANSLATION,
                        VisionConstants.CAMERA_FRONT_ROTATION));
    }

    public CorePigeon2 getPigeon2() {
        return (CorePigeon2) swerveDrive.getGyro().getIMU();
    }

    public void orientModuleOrientationsForPath(Path path) {
        setModuleOrientations(path.getInitialModuleDirection());
    }

    public FollowPath.Builder getPathBuilder() {
        return pathBuilder;
    }

    public boolean isRobotXFacingFieldX() {
        final var currentRot = getRobotPose().getRotation().getDegrees();
        return !MathUtil.isNear(90.0, Math.abs(currentRot) % 180, 45.0);
    }

    public ShooterCalculator getShooterCalculator() {
        return shooterCalculator;
    }

    private enum SOTMHubLockType {
        ANGLE_LOCK("Angle Lock"),
        VELOCITY_FF("Velocity FF"),
        ANGLE_LOCK_AND_VELOCITY_FF("Both"),
        ;

        private final String dashboardName;

        SOTMHubLockType(String dashboardName) {
            this.dashboardName = dashboardName;
        }

        public String getDashboardName() {
            return dashboardName;
        }

        public static SOTMHubLockType fromDashboardName(String dashboardName) {
            return Arrays.stream(values())
                    .filter(type -> type.getDashboardName().equals(dashboardName))
                    .findFirst()
                    .orElseThrow(() -> new IllegalArgumentException("Invalid dashboard name: " + dashboardName));
        }
    }
}
