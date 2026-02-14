package org.pennridge.robotics.frc.manager;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.awt.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import org.pennridge.robotics.frc.Robot;
import org.pennridge.robotics.frc.util.enums.Constants.FieldConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

// Thanks to https://gitlab.com/ironclad_code/ironclad-2024
@NullMarked
public class VisionManager {

    /** Photon Vision Simulation */
    private @Nullable VisionSystemSim visionSim;
    /** Current pose from the pose estimator using wheel odometry. */
    private final Supplier<Pose2d> currentPose;
    /** Field from {@link SwerveDrive#field} */
    private final Field2d field2d;

    /**
     * Constructor for the Vision class.
     *
     * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
     * @param field Current field, should be {@link SwerveDrive#field}
     */
    public VisionManager(Supplier<Pose2d> currentPose, Field2d field) {
        this.currentPose = currentPose;
        this.field2d = field;

        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("Vision");
            visionSim.addAprilTags(FieldConstants.APRIL_TAGS);

            for (Camera camera : Camera.allCameras()) {
                camera.addToVisionSim(visionSim);
            }

            openSimCameraViews();
        }
    }

    /**
     * Calculates a target pose relative to an AprilTag on the field.
     *
     * @param aprilTag The ID of the AprilTag.
     * @param robotOffset The offset {@link Transform2d} of the robot to apply to the pose for the robot to position
     *     itself correctly.
     * @return The target pose of the AprilTag.
     */
    public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
        Optional<Pose3d> aprilTagPose3d = FieldConstants.APRIL_TAGS.getTagPose(aprilTag);
        if (aprilTagPose3d.isPresent()) {
            return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
        } else {
            throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + FieldConstants.APRIL_TAGS);
        }
    }

    /**
     * Update the pose estimation inside of {@link SwerveDrive} with all the given poses.
     *
     * @param swerveDrive {@link SwerveDrive} instance.
     */
    public void updatePoseEstimation(SwerveDrive swerveDrive) {
        if (SwerveDriveTelemetry.isSimulation
                && swerveDrive.getSimulationDriveTrainPose().isPresent()
                && visionSim != null) {
            /*
             * In the maple-sim, odometry is simulated using encoder values, accounting for factors like skidding and drifting.
             * As a result, the odometry may not always be 100% accurate.
             * However, the vision system should be able to provide a reasonably accurate pose estimation, even when odometry is incorrect.
             * (This is why teams implement a vision system to correct odometry.)
             * Therefore, we must ensure that the actual robot pose is provided in the simulator when updating the vision simulation during the simulation.
             */
            visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
        }
        for (Camera camera : Camera.allCameras()) {
            EstimatedRobotPose poseEst = getEstimatedGlobalPose(camera);
            if (poseEst != null) {
                swerveDrive.addVisionMeasurement(
                        poseEst.estimatedPose.toPose2d(), poseEst.timestampSeconds, camera.curStdDevs);
            }
        }
    }

    /**
     * Generates the estimated robot pose. Returns empty if:
     *
     * <ul>
     *   <li>No Pose Estimates could be generated
     *   <li>The generated pose estimate was considered not accurate
     * </ul>
     *
     * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to create the estimate
     */
    public @Nullable EstimatedRobotPose getEstimatedGlobalPose(Camera camera) {
        EstimatedRobotPose poseEst = camera.getEstimatedGlobalPose();
        if (Robot.isSimulation() && visionSim != null) {
            Field2d debugField = visionSim.getDebugField();
            // Uncomment to enable outputting of vision targets in sim.
            if (poseEst != null) {
                debugField.getObject("VisionEstimation").setPose(poseEst.estimatedPose.toPose2d());
            } else {
                debugField.getObject("VisionEstimation").setPoses();
            }
        }
        return poseEst;
    }

    /**
     * Get distance of the robot from the AprilTag pose.
     *
     * @param id AprilTag ID
     * @return Distance
     */
    public @Nullable Distance getDistanceFromAprilTag(int id) {
        Optional<Pose3d> tag = FieldConstants.APRIL_TAGS.getTagPose(id);
        return tag.map(pose3d -> Meters.of(PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())))
                .orElse(null);
    }

    /**
     * Get tracked target from a camera of AprilTag ID
     *
     * @param id AprilTag ID
     * @param camera Camera to check.
     * @return Tracked target.
     */
    public @Nullable PhotonTrackedTarget getTargetFromAprilTag(int id, Camera camera) {
        for (PhotonPipelineResult result : camera.resultsList) {
            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    if (target.getFiducialId() == id) {
                        return target;
                    }
                }
            }
        }
        return null;
    }

    /**
     * Vision simulation.
     *
     * @return Vision Simulation
     */
    public @Nullable VisionSystemSim getVisionSim() {
        return visionSim;
    }

    /** Open up the photon vision camera streams on the localhost, assumes running photon vision on localhost. */
    private void openSimCameraViews() {
        if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
            /*try {
                Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
                Desktop.getDesktop().browse(new URI("http://localhost:1184/"));
                Desktop.getDesktop().browse(new URI("http://localhost:1186/"));
            } catch (IOException | URISyntaxException e) {
                e.printStackTrace();
            }*/
        }
    }

    /** Update the {@link Field2d} to include tracked targets/ */
    public void updateVisionField() {
        List<PhotonTrackedTarget> targets = new ArrayList<>();
        for (Camera camera : Camera.allCameras()) {
            if (!camera.resultsList.isEmpty()) {
                PhotonPipelineResult latest = camera.resultsList.get(0);
                if (latest.hasTargets()) {
                    targets.addAll(latest.targets);
                }
            }
        }

        List<Pose2d> poses = new ArrayList<>();
        for (PhotonTrackedTarget target : targets) {
            if (FieldConstants.APRIL_TAGS.getTagPose(target.getFiducialId()).isPresent()) {
                Pose2d targetPose = FieldConstants.APRIL_TAGS
                        .getTagPose(target.getFiducialId())
                        .get()
                        .toPose2d();
                poses.add(targetPose);
            }
        }

        field2d.getObject("tracked targets").setPoses(poses);
    }

    /** Camera Enum to select each camera */
    public static class Camera {
        // Center Camera
        // The std devs are currently fake values
        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
        public static Camera CENTER = new Camera(
                "center",
                new Rotation3d(0, Units.degreesToRadians(18), 0),
                new Translation3d(
                        Units.inchesToMeters(-4.628), Units.inchesToMeters(-10.687), Units.inchesToMeters(16.129)),
                VecBuilder.fill(4, 4, 8),
                VecBuilder.fill(0.5, 0.5, 1));

        public static List<Camera> allCameras() {
            return List.of(CENTER);
        }

        /** Latency alert to use when high latency is detected. */
        public final Alert latencyAlert;
        /** Camera instance for comms. */
        public final PhotonCamera camera;
        /** Pose estimator for camera. */
        public final PhotonPoseEstimator poseEstimator;
        /** Standard Deviation for single tag readings for pose estimation. */
        private final Matrix<N3, N1> singleTagStdDevs;
        /** Standard deviation for multi-tag readings for pose estimation. */
        private final Matrix<N3, N1> multiTagStdDevs;
        /** Transform of the camera rotation and translation relative to the center of the robot */
        private final Transform3d robotToCamTransform;
        /** Current standard deviations used. */
        public Matrix<N3, N1> curStdDevs;
        /** Estimated robot pose. */
        public @Nullable EstimatedRobotPose estimatedRobotPose = null;

        /** Simulated camera instance which only exists during simulations. */
        public @Nullable PhotonCameraSim cameraSim;
        /** Results list to be updated periodically and cached to avoid unnecessary queries. */
        public List<PhotonPipelineResult> resultsList = new ArrayList<>();

        /**
         * Construct a Photon Camera class with help. Standard deviations are fake values; experiment and determine
         * estimation noise on an actual robot.
         *
         * @param name Name of the PhotonVision camera found in the PV UI.
         * @param robotToCamRotation {@link Rotation3d} of the camera.
         * @param robotToCamTranslation {@link Translation3d} relative to the center of the robot.
         * @param singleTagStdDevs Single AprilTag standard deviations of estimated poses from the camera.
         * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated poses from the camera.
         */
        Camera(
                String name,
                Rotation3d robotToCamRotation,
                Translation3d robotToCamTranslation,
                Matrix<N3, N1> singleTagStdDevs,
                Matrix<N3, N1> multiTagStdDevsMatrix) {
            latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

            camera = new PhotonCamera(name);

            // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
            robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

            poseEstimator = new PhotonPoseEstimator(FieldConstants.APRIL_TAGS, robotToCamTransform);
            // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
            // poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

            this.singleTagStdDevs = singleTagStdDevs;
            this.multiTagStdDevs = multiTagStdDevsMatrix;

            this.curStdDevs = singleTagStdDevs; // default

            if (Robot.isSimulation()) {
                SimCameraProperties cameraProp = new SimCameraProperties();
                // A 640 x 480 camera with a 100-degree diagonal FOV.
                cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
                // Approximate detection noise with average and standard deviation error in pixels.
                cameraProp.setCalibError(0.25, 0.08);
                // Set the camera image capture framerate (Note: this is limited by robot loop rate).
                cameraProp.setFPS(30);
                // The average and standard deviation in milliseconds of image data latency.
                cameraProp.setAvgLatencyMs(35);
                cameraProp.setLatencyStdDevMs(5);

                cameraSim = new PhotonCameraSim(camera, cameraProp);
                cameraSim.enableDrawWireframe(true);
            }
        }

        /**
         * Add camera to {@link VisionSystemSim} for simulated photon vision.
         *
         * @param systemSim {@link VisionSystemSim} to use.
         */
        public void addToVisionSim(VisionSystemSim systemSim) {
            if (Robot.isSimulation()) {
                systemSim.addCamera(cameraSim, robotToCamTransform);
            }
        }

        /**
         * Get the result with the least ambiguity from the best tracked target within the Cache. This may not be the
         * most recent result!
         *
         * @return The result in the cache with the least ambiguous best tracked target. This is not the most recent
         *     result!
         */
        public @Nullable PhotonPipelineResult getBestCachedResult() {
            if (resultsList.isEmpty()) {
                return null;
            }

            PhotonPipelineResult bestResult = resultsList.get(0);
            double ambiguity = bestResult.getBestTarget().getPoseAmbiguity();
            double currentAmbiguity;
            for (PhotonPipelineResult result : resultsList) {
                currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
                if (currentAmbiguity < ambiguity && currentAmbiguity > 0) {
                    bestResult = result;
                    ambiguity = currentAmbiguity;
                }
            }
            return bestResult;
        }

        /**
         * Get the latest result from the current cache.
         *
         * @return Empty optional if nothing is found. Latest result if something is there.
         */
        public @Nullable PhotonPipelineResult getLatestCachedResult() {
            return resultsList.isEmpty() ? null : resultsList.get(0);
        }

        /**
         * Get the estimated robot pose. Updates the current robot pose estimation, standard deviations, and flushes the
         * cache of results.
         *
         * @return Estimated pose.
         */
        public @Nullable EstimatedRobotPose getEstimatedGlobalPose() {
            updateUnreadResults();
            return estimatedRobotPose;
        }

        /** Update the latest results, cached with a maximum refresh rate of 1req/15ms. Sorts the list by timestamp. */
        private void updateUnreadResults() {
            double mostRecentTimestamp =
                    resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();

            for (PhotonPipelineResult result : resultsList) {
                mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
            }

            resultsList = (Robot.isReal() || cameraSim == null)
                    ? camera.getAllUnreadResults()
                    : cameraSim.getCamera().getAllUnreadResults();
            resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) ->
                    a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1);
            if (!resultsList.isEmpty()) {
                updateEstimatedGlobalPose();
            }
        }

        /**
         * The latest estimated robot pose on the field from vision data. This may be empty. This should only be called
         * once per loop.
         *
         * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
         * {@link Camera#updateEstimationStdDevs}
         */
        private void updateEstimatedGlobalPose() {
            EstimatedRobotPose visionEst = null;
            for (var change : resultsList) {
                visionEst = poseEstimator.estimateCoprocMultiTagPose(change).orElseGet(() -> poseEstimator
                        .estimateLowestAmbiguityPose(change)
                        .orElse(null));
                updateEstimationStdDevs(visionEst, change.getTargets());
            }
            estimatedRobotPose = visionEst;
        }

        /**
         * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard deviations
         * based on the number of tags, estimation strategy, and distance from the tags.
         *
         * @param estimatedPose The estimated pose to guess standard deviations for.
         * @param targets All targets in this camera frame
         */
        private void updateEstimationStdDevs(
                @Nullable EstimatedRobotPose estimatedPose, List<PhotonTrackedTarget> targets) {
            if (estimatedPose == null) {
                // No pose input. Default to single-tag std devs
                curStdDevs = singleTagStdDevs;
                return;
            }
            // Pose present. Start running Heuristic
            var estStdDevs = singleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) {
                    continue;
                }
                numTags++;
                avgDist += tagPose.get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = singleTagStdDevs;
                return;
            }
            // One or more tags visible, run the full heuristic.
            avgDist /= numTags;
            // Decrease std devs if multiple targets are visible
            if (numTags > 1) {
                estStdDevs = multiTagStdDevs;
            }
            // Increase std devs based on (average) distance
            if (numTags == 1 && avgDist > 4) {
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
                estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
            }
            curStdDevs = estStdDevs;
        }
    }
}
