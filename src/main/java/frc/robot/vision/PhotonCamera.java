package frc.robot.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.enums.Constants.FieldConstants;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * {@code PhotonCamera} encapsulates a PhotonVision camera used for estimating the robot’s field pose using AprilTags.
 *
 * <p>This class integrates PhotonVision’s {@link PhotonPoseEstimator} with FRC odometry by converting camera
 * observations into {@link PoseEstimate} objects that can be fused with other localization sources (e.g., swerve
 * odometry, gyro).
 *
 * <p>It also publishes telemetry to NetworkTables for visualization in dashboards such as Shuffleboard or
 * AdvantageScope.
 *
 * <h3>Features:</h3>
 *
 * <ul>
 *   <li>Automatic handling of single- and multi-tag pose strategies.
 *   <li>Support for simulation via {@link PhotonCameraSim} and {@link SimCameraProperties}.
 *   <li>Dynamic disabling of estimation via {@link BooleanSupplier} conditions.
 *   <li>NetworkTables telemetry for estimated pose, tracked targets, and tag corners.
 * </ul>
 */
// Credits to https://gitlab.com/ironclad_code/ironclad-2026/
@NullMarked
public class PhotonCamera extends Camera {

    /** Physical PhotonVision camera instance. */
    private final org.photonvision.PhotonCamera camera;

    /** Simulation wrapper for the Photon camera. */
    private final PhotonCameraSim simCamera;

    /** Camera property configuration used in simulation mode. */
    private final SimCameraProperties simProperties;

    /** PhotonVision pose estimator handling field-relative position estimation. */
    private final PhotonPoseEstimator poseEstimator;

    /** Transform representing the camera’s position and orientation on the robot. */
    private final Transform3d robotToCam;

    // Telemetry publishers
    private final StructPublisher<Pose2d> posePublisher;
    private final StructArrayPublisher<Pose3d> trackedTargetsPublisher;
    private final StructArrayPublisher<Translation2d> trackedCornersPublisher;
    private final DoublePublisher stdDevsPublisher;
    private final DoublePublisher confidencePublisher;

    /**
     * Constructs a {@code PhotonCamera} with full configuration
     *
     * @param name The name of the PhotonVision camera in the client.
     * @param robotToCamTranslation The translation offset (in meters) from the robot origin to the camera lens center.
     * @param robotToCamRotation The rotation offset of the camera relative to the robot frame.
     */
    public PhotonCamera(String name, Translation3d robotToCamTranslation, Rotation3d robotToCamRotation) {
        camera = new org.photonvision.PhotonCamera(name);
        final var topicPrefix = "Vision/" + name + "/";
        posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(topicPrefix + "Estimated Pose", Pose2d.struct)
                .publish();
        trackedTargetsPublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic(topicPrefix + "Tracked Targets", Pose3d.struct)
                .publish();
        trackedCornersPublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic(topicPrefix + "Corners", Translation2d.struct)
                .publish();
        stdDevsPublisher = NetworkTableInstance.getDefault()
                .getDoubleTopic(topicPrefix + "Standard Deviations")
                .publish();
        confidencePublisher = NetworkTableInstance.getDefault()
                .getDoubleTopic(topicPrefix + "Confidence")
                .publish();

        // Simulation configuration
        simProperties = new SimCameraProperties();
        simProperties.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        simProperties.setCalibError(0.25, 0.08);
        simProperties.setFPS(30);
        simProperties.setAvgLatencyMs(35);
        simProperties.setLatencyStdDevMs(5);

        simCamera = new PhotonCameraSim(camera, simProperties);
        robotToCam = new Transform3d(robotToCamTranslation, robotToCamRotation);

        poseEstimator = new PhotonPoseEstimator(FieldConstants.APRIL_TAGS, robotToCam);

        simCamera.enableDrawWireframe(true);
    }

    /** @return The {@link PhotonCameraSim} used for simulation. */
    public PhotonCameraSim getSimCamera() {
        return simCamera;
    }

    /** @return The {@link SimCameraProperties} used by the simulated camera. */
    public SimCameraProperties getSimProperties() {
        return simProperties;
    }

    /** @return The {@link Transform3d} from the robot origin to the camera lens. */
    public Transform3d getRobotToCam() {
        return robotToCam;
    }

    /**
     * Updates the camera’s pose estimation using the latest unread PhotonVision results.
     *
     * <p>This method should be called periodically (e.g., every robot loop). It retrieves the latest
     * {@link PhotonPipelineResult}, runs the {@link PhotonPoseEstimator}, and publishes telemetry if successful.
     *
     * @return An {@link Optional} containing a {@link PoseEstimate} if a valid estimate was produced. Empty if no valid
     *     data or disabled.
     */
    @Override
    public @Nullable PoseEstimate update() {
        PoseEstimate estimatedPose = null;
        var results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            return estimatedPose;
        }
        PhotonPipelineResult latestResult = results.get(results.size() - 1);
        Optional<EstimatedRobotPose> photonEstimatedPose;

        int tagCount = countFiducials(latestResult);

        if (tagCount >= 2) {
            photonEstimatedPose = poseEstimator.estimateCoprocMultiTagPose(latestResult);
            if (photonEstimatedPose.isEmpty()) {
                photonEstimatedPose = poseEstimator.estimateLowestAmbiguityPose(latestResult);
            }
        } else {
            photonEstimatedPose = poseEstimator.estimateLowestAmbiguityPose(latestResult);
        }

        if (photonEstimatedPose.isPresent()) {
            EstimatedRobotPose photonPose = photonEstimatedPose.get();

            // Apply filters before accepting the estimate
            if (isPoseReasonable(photonPose)) {
                final var stdDevs = getEstimationStdDevs(photonPose);
                if (stdDevs != null) {
                    estimatedPose =
                            new PoseEstimate(photonPose.estimatedPose.toPose2d(), photonPose.timestampSeconds, stdDevs);
                    posePublisher.set(photonPose.estimatedPose.toPose2d());
                    stdDevsPublisher.set(stdDevs.get(0, 0));
                    publishGlobalStdDev(stdDevs.get(0, 0));
                }
            }
        }

        // Publish telemetry for visible tags
        if (!latestResult.hasTargets()) {
            trackedTargetsPublisher.set(new Pose3d[0]);
            trackedCornersPublisher.set(new Translation2d[0]);
            return estimatedPose;
        }

        Pose3d[] posesArray = new Pose3d[latestResult.targets.size()];
        Translation2d[] corners = new Translation2d[latestResult.targets.size() * 4];
        int cornerIndex = 0;

        for (int i = 0; i < posesArray.length; i++) {
            PhotonTrackedTarget target = latestResult.targets.get(i);
            posesArray[i] =
                    FieldConstants.APRIL_TAGS.getTagPose(target.getFiducialId()).orElse(new Pose3d());

            // Flatten each tag's 4 detected corners
            for (int j = 0; j < 4; j++) {
                var pt = target.getDetectedCorners().get(j);
                corners[cornerIndex++] = new Translation2d(pt.x, pt.y);
            }
        }

        trackedTargetsPublisher.set(posesArray);
        trackedCornersPublisher.set(corners);

        return estimatedPose;
    }

    /**
     * Computes standard deviation estimates based on the number of visible tags and their distance.
     *
     * @param poseEst The estimated robot pose returned by PhotonVision.
     * @return A 3×1 matrix representing (x, y, rotation) standard deviations.
     */
    private @Nullable Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose poseEst) {
        int numTags = 0;
        double avgDist = 0;

        for (var tgt : poseEst.targetsUsed) {
            var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;

            numTags++;
            avgDist += PhotonUtils.getDistanceToPose(
                    poseEst.estimatedPose.toPose2d(), tagPose.get().toPose2d());
        }

        if (numTags == 0) return null;
        avgDist /= numTags;

        return getEstimationStdDevs(numTags, Meters.of(avgDist), false);
    }

    /**
     * Adds a heading measurement to the pose estimator, allowing for gyro fusion.
     *
     * @param rotation The robot’s current {@link Rotation2d} heading.
     */
    public void addRotation(Rotation2d rotation) {
        poseEstimator.addHeadingData(Timer.getFPGATimestamp(), rotation);
    }

    /** Saves an image snapshot of the current camera output, useful for debugging. */
    public void takeOutputSnapshot() {
        camera.takeOutputSnapshot();
    }

    private static int countFiducials(PhotonPipelineResult result) {
        int count = 0;
        for (var t : result.targets) {
            if (t.getFiducialId() >= 0) count++;
        }
        return count;
    }

    /**
     * Validates whether a pose estimate is reasonable before accepting it. Filters out: - Ambiguous single-tag
     * estimates - Poses with unrealistic heights - Poses outside field boundaries
     *
     * @param photonPose The estimated robot pose from PhotonVision
     * @return true if the estimate passes all sanity checks
     */
    private boolean isPoseReasonable(EstimatedRobotPose photonPose) {
        var targets = photonPose.targetsUsed;
        int numTags = (int) targets.stream().filter(t -> t.getFiducialId() >= 0).count();

        if (numTags == 1) {
            PhotonTrackedTarget target = targets.get(0);
            double ambiguity = target.getPoseAmbiguity();
            if (ambiguity > 0.2) {
                return false;
            }
        }

        Pose3d pose3d = photonPose.estimatedPose;
        double robotHeightM = pose3d.getZ();
        if (Math.abs(robotHeightM) > 0.5) {
            return false;
        }

        Pose2d pose2d = pose3d.toPose2d();
        double x = pose2d.getX();
        double y = pose2d.getY();

        double fieldLength = FieldConstants.FIELD_LENGTH_X.in(Meters);
        double fieldWidth = FieldConstants.FIELD_WIDTH_Y.in(Meters);
        final double FIELD_MARGIN = 0.5;

        return !(x < -FIELD_MARGIN)
                && !(x > fieldLength + FIELD_MARGIN)
                && !(y < -FIELD_MARGIN)
                && !(y > fieldWidth + FIELD_MARGIN);
    }
}
