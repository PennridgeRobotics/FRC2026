package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.core.CorePigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.util.BumpManager;
import frc.robot.util.SlewRateLimiter2d;
import frc.robot.util.dashboard.PIDSendable;
import frc.robot.util.dashboard.PIDSendable.PIDValues;
import frc.robot.util.enums.Constants.BLineConstants;
import frc.robot.util.enums.Constants.ControllerConstants;
import frc.robot.util.enums.Constants.DriveConstants;
import frc.robot.util.enums.Constants.VisionConstants;
import frc.robot.vision.PhotonCamera;
import frc.robot.vision.VisionManager;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
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

    private boolean forceNormalDriveMode = false;
    private boolean lockYawTowardsVelocity = false;

    private final Trigger forceNormalDriveModeTrigger = new Trigger(() -> forceNormalDriveMode);

    private final PIDController bLineTranslationPID = new PIDController(5.0, 0, 0);
    private final PIDController bLineRotationPID = new PIDController(3.0, 0, 0);
    private final PIDController bLineCrossTrackPID = new PIDController(2.0, 0, 0);
    private final FollowPath.Builder pathBuilder;

    private final SlewRateLimiter2d linearDriveLimiter =
            new SlewRateLimiter2d(DriveConstants.MAX_LINEAR_ACCELERATION.in(MetersPerSecondPerSecond));

    @SuppressWarnings("StaticAssignmentInConstructor")
    public SwerveSubsystem() throws IOException {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        swerveDrive = new SwerveParser(
                        new File(Filesystem.getDeployDirectory(), DriveConstants.SWERVE_CONFIG_DIRECTORY))
                .createSwerveDrive(
                        DriveConstants.MAX_LINEAR_SPEED.in(MetersPerSecond),
                        new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)), Rotation2d.kZero));
        // swerveDrive.setHeadingCorrection(false); // enable this after testing/tuning PID
        // swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // disable for simulations
        // swerveDrive.setAngularVelocityCompensation(true, true, 0.1); // may need to adjust; see docs
        // swerveDrive.useExternalFeedbackSensor();
        // swerveDrive.setModuleEncoderAutoSynchronize(false, 1); // can set to true, but I want to test
        // swerveDrive.setMotorIdleMode(true);

        // swerveDrive.setHeadingCorrection(false);
        // swerveDrive.setCosineCompensator(false);
        // swerveDrive.setAngularVelocityCompensation(false, false, 0.1);
        // swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

        bumpManager = new BumpManager(
                getPigeon2(), swerveDrive::getGyroRotation3d, this::getRobotPose, forceNormalDriveModeTrigger);

        setupVisionManager();
        pathBuilder = setupBLine();
        initSmartDashboard();
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
            if (lockYawTowardsVelocity && !forceNormalDriveMode) {
                chassisSpeeds.omegaRadiansPerSecond = getVelocityAngle(
                                MetersPerSecond.of(chassisSpeeds.vxMetersPerSecond),
                                MetersPerSecond.of(chassisSpeeds.vyMetersPerSecond))
                        .getRadians();
            }
            swerveDrive.drive(chassisSpeeds, false, new Translation2d());
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
        if (DriverStation.Alliance.Red.equals(DriverStation.getAlliance().orElse(null))) {
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
        final var shouldFlip = alliance.get() == DriverStation.Alliance.Red;
        final var adjustedXVelocity = shouldFlip ? xVelocity.unaryMinus() : xVelocity;
        final var adjustedYVelocity = shouldFlip ? yVelocity.unaryMinus() : yVelocity;
        final Translation2d limitedLinearVelocity = linearDriveLimiter.calculate(
                adjustedXVelocity.in(MetersPerSecond), adjustedYVelocity.in(MetersPerSecond));
        final AngularVelocity finalAngularVelocity =
                bumpManager.isBumpLockEnabledTrigger().getAsBoolean()
                        ? getTargetAngularVelocity(bumpManager.getBumpLockAngle())
                        : ((lockYawTowardsVelocity && !forceNormalDriveMode)
                                ? getTargetAngularVelocity(getVelocityAngle(
                                        MetersPerSecond.of(limitedLinearVelocity.getX()),
                                        MetersPerSecond.of(limitedLinearVelocity.getY())))
                                : angularVelocity);
        swerveDrive.driveFieldOriented(new ChassisSpeeds(
                limitedLinearVelocity.getX(), limitedLinearVelocity.getY(), finalAngularVelocity.in(RadiansPerSecond)));
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

    public Command straightenWheelsCommand() {
        return run(() -> setModuleOrientations(Rotation2d.kZero));
    }

    public Command lockYawTowardsVelocity(final boolean lockEnabled) {
        return runOnce(() -> lockYawTowardsVelocity = lockEnabled);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public Command setManualBumpLock(final boolean locked) {
        return bumpManager.setManualBumpLock(locked);
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
        final var scaled = Math.pow(withDeadband, 3);
        return getMaximumChassisVelocity().times(scaled);
    }

    private AngularVelocity joystickToAngularVelocity(final double input) {
        final var withDeadband =
                MathUtil.applyDeadband(input, ControllerConstants.DRIVE_MIN_INPUT, ControllerConstants.DRIVE_MAX_INPUT);
        final var scaled = Math.pow(withDeadband, 3);
        return getMaximumChassisAngularVelocity().times(scaled);
    }

    private void setModuleOrientations(Rotation2d rotation) {
        final var states = new SwerveModuleState[swerveDrive.getModules().length];
        Arrays.fill(states, new SwerveModuleState(0, rotation));
        swerveDrive.setModuleStates(states, false);
    }

    private FollowPath.Builder setupBLine() {
        Path.setDefaultGlobalConstraints(BLineConstants.GLOBAL_CONSTRAINTS);
        SmartDashboard.putData("BLine Translation PID", new PIDSendable(bLineTranslationPID, PIDSendable.Type.PID));
        SmartDashboard.putData("BLine Rotation PID", new PIDSendable(bLineRotationPID, PIDSendable.Type.PID));
        SmartDashboard.putData("BLine Cross Track PID", new PIDSendable(bLineCrossTrackPID, PIDSendable.Type.PID));
        return new FollowPath.Builder(
                        this,
                        this::getRobotPose,
                        swerveDrive::getRobotVelocity,
                        swerveDrive::drive,
                        bLineTranslationPID,
                        bLineRotationPID,
                        bLineCrossTrackPID)
                .withDefaultShouldFlip()
                .withPoseReset(this::resetPose);
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(
                "Swerve Subsystem",
                builder -> builder.addStringProperty(
                        "Bump Status",
                        () -> {
                            if (!bumpManager.isRawBumpLockEnabledTrigger().getAsBoolean())
                                return Color.kLime.toHexString();
                            if (bumpManager.isOnBumpTrigger().getAsBoolean()) return Color.kRed.toHexString();
                            return Color.kYellow.toHexString(); // in bump area, but not on the bump itself
                        },
                        null));
        SmartDashboard.putData(
                "Swerve Controller Heading PID",
                new PIDSendable(
                        swerveDrive.swerveController.thetaController,
                        PIDSendable.Type.PID,
                        PIDValues.from(swerveDrive.swerveController.thetaController)));
    }

    private void resetPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    private SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    private Pose2d getRobotPose() {
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
        visionManager.addCamera(new PhotonCamera(
                VisionConstants.CAMERA_1_NAME,
                VisionConstants.CAMERA_1_TRANSLATION,
                VisionConstants.CAMERA_1_ROTATION));
    }

    private CorePigeon2 getPigeon2() {
        return (CorePigeon2) swerveDrive.getGyro().getIMU();
    }
}
