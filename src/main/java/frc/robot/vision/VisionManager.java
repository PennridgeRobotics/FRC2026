package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;
import frc.robot.util.enums.Constants.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import org.photonvision.simulation.VisionSystemSim;
import swervelib.SwerveDrive;

// Credits to https://gitlab.com/ironclad_code/ironclad-2026/
@NullMarked
public class VisionManager {
    private final VisionSystemSim visionSim;
    private final List<Camera> cameras;
    private final Supplier<Pose2d> getSimDrivetrainPose;

    /**
     * Creates a new Pose Estimator
     *
     * @param getSimPose The Pose of the robot in simulation only. This should be separate from the physical pose
     */
    public VisionManager(Supplier<Pose2d> getSimPose) {
        visionSim = new VisionSystemSim("Vision Sim");
        visionSim.addAprilTags(FieldConstants.APRIL_TAGS);
        this.cameras = new ArrayList<>();
        this.getSimDrivetrainPose = getSimPose;
    }

    /** Adds a camera to the Pose Estimator */
    public void addCamera(Camera camera) {
        cameras.add(camera);

        if (camera instanceof PhotonCamera cam) {
            visionSim.addCamera(cam.getSimCamera(), cam.getRobotToCam());
        }
    }

    /** Adds a list of cameras to the Pose Estimator */
    public void addCameras(Camera... cameras) {
        for (Camera camera : cameras) {
            addCamera(camera);
        }
    }

    /**
     * Update the robots pose from vision
     *
     * @param swerveDrive the YAGSL SwerveDrive object
     */
    public void updatePoseEstimation(SwerveDrive swerveDrive) {
        // update pose estimations from photon cameras
        if (Robot.isSimulation()) visionSim.update(getSimDrivetrainPose.get());
        for (Camera camera : cameras) {
            if (camera instanceof LimelightCamera limelightCamera) {
                limelightCamera.setRobotOrientation(swerveDrive.getOdometryHeading());
            }
            PoseEstimate poseEstimate = camera.update();
            if (poseEstimate != null) {
                swerveDrive.addVisionMeasurement(
                        poseEstimate.poseEstimate, poseEstimate.timeStamp, poseEstimate.stdDevs);
            }
        }
    }
}
