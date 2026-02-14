package org.pennridge.robotics.frc.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.jspecify.annotations.NullMarked;

// Credits to https://gitlab.com/ironclad_code/ironclad-2026/
@NullMarked
public class PoseEstimate {
    protected Pose2d poseEstimate;
    protected double timeStamp;
    protected Matrix<N3, N1> stdDevs;

    public PoseEstimate(Pose2d poseEstimate, double timeStamp, Matrix<N3, N1> stdDevs) {
        this.timeStamp = timeStamp;
        this.poseEstimate = poseEstimate;
        this.stdDevs = stdDevs;
    }
}
