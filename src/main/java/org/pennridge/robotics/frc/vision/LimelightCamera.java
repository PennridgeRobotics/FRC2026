package org.pennridge.robotics.frc.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import java.util.Optional;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import org.pennridge.robotics.frc.util.enums.Constants.FieldConstants;
import org.pennridge.robotics.frc.util.lib.LimelightHelpers;

/**
 * {@code LimelightCamera} provides an {@link Camera} implementation backed by a Limelight running AprilTag pose
 * estimation.
 *
 * <p>This class fetches robot pose estimates from the Limelight (either standard WPI Blue or MegaTag2 pipeline),
 * converts them to {@link PoseEstimate} for downstream fusion, and publishes useful telemetry to NetworkTables:
 *
 * <ul>
 *   <li>Estimated field pose (Pose2d)
 *   <li>Tracked tag poses (Pose3d[] from the loaded field layout)
 *   <li>MegaTag2 enabled flag
 * </ul>
 *
 * Disable conditions can be registered to prevent pose updates from being consumed by the rest of the system, though
 * the raw telemetry can still be published.
 */
// Credits to https://gitlab.com/ironclad_code/ironclad-2026/
@NullMarked
public class LimelightCamera extends Camera {
    /** Publishes the most recent estimated field pose to NetworkTables. */
    private final StructPublisher<Pose2d> posePublisher;

    /** Publishes the current set of visible/used tag field poses (from the field layout). */
    private final StructArrayPublisher<Pose3d> trackedTargetsPublisher;

    /** Publishes whether MegaTag2 processing is enabled for this camera. */
    private final BooleanPublisher megaTag2Publisher;

    /** Publishes the standard deviations being used for pose estimations */
    private final DoublePublisher stdDevsPublisher;

    /** Limelight name as configured on the device and used by {@link LimelightHelpers}. */
    private final String name;

    /** Flag controlling whether MegaTag2 mode is used for pose estimation. */
    private final boolean megaTag2;

    /**
     * Creates a new {@code LimelightCamera} with full configuration.
     *
     * @param name The Limelight name (must match Limelight device configuration).
     * @param megaTag2 If {@code true}, MegaTag2 mode is considered enabled for this camera.
     */
    public LimelightCamera(String name, boolean megaTag2) {
        this.name = name;
        this.megaTag2 = megaTag2;
        final var topicPrefix = "Vision/" + name + "/";
        posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(topicPrefix + "Estimated Pose", Pose2d.struct)
                .publish();
        trackedTargetsPublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic(topicPrefix + "Tracked Targets", Pose3d.struct)
                .publish();
        megaTag2Publisher = NetworkTableInstance.getDefault()
                .getBooleanTopic(topicPrefix + "MegaTag2")
                .publish();
        stdDevsPublisher = NetworkTableInstance.getDefault()
                .getDoubleTopic(topicPrefix + "Standard Deviations")
                .publish();
    }

    /**
     * Polls the Limelight for the latest pose estimate and returns it if available.
     *
     * <p>Behavior:
     *
     * <ul>
     *   <li>Reads a pose estimate from Limelight via {@link LimelightHelpers}.
     *   <li>Publishes the MegaTag2 enable flag to NetworkTables.
     *   <li>If a pose is present, wraps it in a {@link PoseEstimate} with a fixed process noise of (0.5, 0.5, 0.5) for
     *       (x, y, theta) and publishes telemetry for the pose and currently tracked tag field poses.
     *   <li>Returns {@link Optional#empty()} when no pose is available.
     * </ul>
     *
     * @return An {@link Optional} containing the latest {@link PoseEstimate}, or empty if no pose is available.
     */
    @Override
    public @Nullable PoseEstimate update() {
        PoseEstimate estimatedPose = null;
        // Note: The 'megaTag2' flag controls the choice of helper here.
        LimelightHelpers.PoseEstimate llPose = megaTag2
                ? LimelightHelpers.getBotPoseEstimate_wpiBlue(name)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        megaTag2Publisher.set(megaTag2);

        if (llPose.tagCount > 0) {
            final var stdDevs = getEstimationStdDevs(llPose);
            if (stdDevs == null) {
                return null;
            }
            // Caller can fuse these with drivetrain odometry using the provided std devs
            estimatedPose = new PoseEstimate(llPose.pose, llPose.timestampSeconds, stdDevs);
            posePublisher.set(llPose.pose);
            trackedTargetsPublisher.set(getTargetPoses(llPose.rawFiducials));
            stdDevsPublisher.set(stdDevs.get(0, 0));
            publishGlobalStdDev(stdDevs.get(0, 0));
        }

        return estimatedPose;
    }

    /**
     * Provides the robot's current yaw to the Limelight to improve pose solving.
     *
     * <p>Internally calls {@link LimelightHelpers#SetRobotOrientation(String, double, double, double, double, double,
     * double)} with all rates set to zero and only yaw supplied.
     *
     * @param rotation Current robot heading as a {@link Rotation2d}.
     */
    public void setRobotOrientation(Rotation2d rotation) {
        LimelightHelpers.SetRobotOrientation(name, rotation.getDegrees(), 0, 0, 0, 0, 0);
    }

    /**
     * Converts a set of raw fiducials returned by the Limelight into field poses using the robot's loaded
     * {@link FieldConstants#APRIL_TAGS}.
     *
     * <p>If no fiducials are present, returns an empty array.
     *
     * @param tags Array of raw fiducials from the Limelight pose's result.
     * @return Array of {@link Pose3d} tag poses from the field layout (same order as input).
     */
    private Pose3d[] getTargetPoses(LimelightHelpers.RawFiducial @Nullable [] tags) {
        if (tags == null || tags.length == 0) {
            return new Pose3d[0];
        }
        Pose3d[] poses = new Pose3d[tags.length];
        for (int i = 0; i < poses.length; i++) {
            poses[i] = FieldConstants.APRIL_TAGS.getTagPose(tags[i].id).orElse(new Pose3d());
        }
        return poses;
    }

    /**
     * Computes standard deviation estimates based on the number of visible tags and their distance.
     *
     * @param poseEst The estimated robot pose returned by LimeLight.
     * @return A 3×1 matrix representing (x, y, rotation) standard deviations.
     */
    private @Nullable Matrix<N3, N1> getEstimationStdDevs(LimelightHelpers.PoseEstimate poseEst) {
        return getEstimationStdDevs(poseEst.tagCount, Meters.of(poseEst.avgTagDist), true);
    }
}
