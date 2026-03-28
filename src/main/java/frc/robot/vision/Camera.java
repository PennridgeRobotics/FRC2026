package frc.robot.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.dashboard.LoggedNetworkBoolean;
import frc.robot.util.enums.Constants.VisionConstants;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;

// Credits to https://gitlab.com/ironclad_code/ironclad-2026/
@NullMarked
public abstract class Camera {

    private static final DoublePublisher stdDevsPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("Vision/All Standard Deviations")
            .publish();
    private final LoggedNetworkBoolean loggedUseInPoseEstimation;

    protected final String cameraName;

    protected Camera(String cameraName, boolean useInPoseEstimationDefault) {
        this.cameraName = cameraName;
        loggedUseInPoseEstimation = new LoggedNetworkBoolean(
                "/Vision/" + cameraName + "/Use in Pose Estimation", useInPoseEstimationDefault);
    }

    public abstract @Nullable PoseEstimate update();

    public boolean useInPoseEstimation() {
        return loggedUseInPoseEstimation.getAsBoolean();
    }

    public Command toggleUseInPoseEstimation() {
        return Commands.runOnce(() -> loggedUseInPoseEstimation.set(!loggedUseInPoseEstimation.getAsBoolean()));
    }

    /**
     * Computes standard deviation estimates based on the number of visible tags and their distance.
     *
     * @return A 3×1 matrix representing (x, y, rotation) standard deviations.
     */
    protected @Nullable Matrix<N3, N1> getEstimationStdDevs(
            int numTags, Distance averageTagDistance, boolean isLimeLight) {
        if (numTags == 0) return null;

        final var singleTagStdDevs = isLimeLight
                ? VisionConstants.LIMELIGHT_SINGLE_TAG_STD_DEVS
                : VisionConstants.PHOTON_SINGLE_TAG_STD_DEVS;
        final var multiTagStdDevs =
                isLimeLight ? VisionConstants.LIMELIGHT_MULTI_TAG_STD_DEVS : VisionConstants.PHOTON_MULTI_TAG_STD_DEVS;

        // Adjust standard deviations
        var estStdDevs = numTags == 1 ? singleTagStdDevs : multiTagStdDevs;

        estStdDevs = estStdDevs.times(1
                + (Math.pow(averageTagDistance.in(Meters), VisionConstants.STD_DEV_DISTANCE_EXPONENT)
                        * VisionConstants.STD_DEV_DISTANCE_MULTIPLIER));
        estStdDevs.set(2, 0, 999999);
        return estStdDevs;
    }

    protected void publishGlobalStdDev(double stdDev) {
        stdDevsPublisher.set(stdDev);
    }

    public String getCameraName() {
        return cameraName;
    }
}
