package org.pennridge.robotics.frc.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import org.pennridge.robotics.frc.util.enums.Constants.ControllerConstants;
import org.pennridge.robotics.frc.util.enums.Constants.DriveConstants;
import org.pennridge.robotics.frc.util.enums.Constants.VisionConstants;
import org.pennridge.robotics.frc.util.lib.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

@NullMarked
public class RealSwerveSubsystem extends SubsystemBase implements SwerveSubsystem {
    protected final SwerveDrive swerveDrive;

    @SuppressWarnings("StaticAssignmentInConstructor")
    public RealSwerveSubsystem() throws IOException {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        swerveDrive = new SwerveParser(
                        new File(Filesystem.getDeployDirectory(), DriveConstants.SWERVE_CONFIG_DIRECTORY))
                .createSwerveDrive(DriveConstants.MAX_LINEAR_SPEED.in(MetersPerSecond));
        swerveDrive.setHeadingCorrection(false);

        initSmartDashboard();
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
        updateVision();
    }

    public Command resetRobotPose(Supplier<Pose2d> resetPose) {
        return runOnce(() -> swerveDrive.resetOdometry(resetPose.get()));
    }

    /**
     * @param xVelocity Positive = towards other alliance
     * @param yVelocity Positive = towards left wall
     * @param angularVelocity Positive = CCW
     */
    public Command driveFieldOrientedCommand(
            final Supplier<LinearVelocity> xVelocity,
            final Supplier<LinearVelocity> yVelocity,
            final Supplier<AngularVelocity> angularVelocity) {
        return run(() -> driveFieldOriented(xVelocity.get(), yVelocity.get(), angularVelocity.get()));
    }

    /**
     * @param xInput Positive = towards other alliance
     * @param yInput Positive = towards left wall
     * @param angularInput Positive = CCW
     */
    public Command driveFieldOrientedCommand(
            final DoubleSupplier xInput, final DoubleSupplier yInput, final DoubleSupplier angularInput) {
        return driveFieldOrientedCommand(
                () -> joystickToLinearVelocity(xInput.getAsDouble()),
                () -> joystickToLinearVelocity(yInput.getAsDouble()),
                () -> joystickToAngularVelocity(angularInput.getAsDouble()));
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
        swerveDrive.driveFieldOriented(new ChassisSpeeds(adjustedXVelocity, adjustedYVelocity, angularVelocity));
    }

    private LinearVelocity joystickToLinearVelocity(final double input) {
        final var withDeadband =
                MathUtil.applyDeadband(input, ControllerConstants.DRIVE_MIN_INPUT, ControllerConstants.DRIVE_MAX_INPUT);
        final var scaled = Math.pow(withDeadband, 3);
        return DriveConstants.MAX_LINEAR_SPEED.times(scaled);
    }

    private AngularVelocity joystickToAngularVelocity(final double input) {
        final var withDeadband =
                MathUtil.applyDeadband(input, ControllerConstants.DRIVE_MIN_INPUT, ControllerConstants.DRIVE_MAX_INPUT);
        final var scaled = Math.pow(withDeadband, 3);
        return DriveConstants.MAX_ANGULAR_SPEED.times(scaled);
    }

    private void updateVision() {
        LimelightHelpers.SetRobotOrientation(
                VisionConstants.LIMELIGHT_NAME, swerveDrive.getYaw().getDegrees(), 0, 0, 0, 0, 0);
        final var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LIMELIGHT_NAME);
        if (Math.abs(swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond)) > 360) {
            return;
        }
        if (estimate == null || estimate.tagCount == 0) {
            return;
        }
        swerveDrive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        swerveDrive.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
    }

    private void initSmartDashboard() {}

    private SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    private Pose2d getRobotPose() {
        return swerveDrive.getPose();
    }

    public Command resetYaw() {
        return runOnce(swerveDrive::zeroGyro);
    }

    @Override
    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        swerveDrive.drive(speeds);
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates) {}

    @Override
    public ChassisSpeeds getMeasuredSpeeds() {
        throw new UnsupportedOperationException();
    }

    @Override
    public Rotation2d getGyroYaw() {
        throw new UnsupportedOperationException();
    }

    @Override
    public Pose2d getPose() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setPose(Pose2d pose) {}

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds) {}

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {}
}
