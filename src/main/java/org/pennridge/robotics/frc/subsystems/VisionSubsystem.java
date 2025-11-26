package org.pennridge.robotics.frc.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.studica.frc.AHRS;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jetbrains.annotations.NotNull;
import org.pennridge.robotics.frc.util.LimelightHelpers;
import org.pennridge.robotics.frc.util.enums.Constants.PhysicalConstants;
import org.pennridge.robotics.frc.util.enums.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private final @NotNull AHRS ahrs;
    private final @NotNull AprilTagFieldLayout field;
    private final @NotNull Transform2d limelightOffset =
            new Transform2d(PhysicalConstants.LIMELIGHT_OFFSET_X.unaryMinus(), Meters.zero(), Rotation2d.kZero);

    public VisionSubsystem(@NotNull AHRS ahrs, @NotNull AprilTagFieldLayout field) {
        this.ahrs = ahrs;
        this.field = field;
    }

    @Override
    public void periodic() {
        updateVision();
    }

    private void updateVision() {
        LimelightHelpers.SetRobotOrientation(
                VisionConstants.LIMELIGHT_NAME,
                ahrs.getYaw(), // use PoseEstimator
                -ahrs.getRate(), // NavX uses CW; FRC uses CCW
                ahrs.getPitch(),
                0,
                ahrs.getRoll(),
                0);
        final LimelightHelpers.PoseEstimate estimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LIMELIGHT_NAME);
        if (Math.abs(ahrs.getRate()) > 360) {
            return;
        }
        if (estimate == null || estimate.tagCount == 0) {
            return;
        }
        final var newPose = estimate.pose; // .plus(limelightOffset); - offset handled by Limelight
        // poseEstimatorManager.addVisionMeasurement(newPose, estimate.timestampSeconds);
    }
}
