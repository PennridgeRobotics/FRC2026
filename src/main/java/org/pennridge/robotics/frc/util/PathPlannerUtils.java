package org.pennridge.robotics.frc.util;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearAcceleration;
import java.util.Arrays;
import org.jetbrains.annotations.NotNull;

public class PathPlannerUtils {
    public static Trajectory pathPlannerTrajectoryToWPILib(final @NotNull PathPlannerTrajectory trajectory) {
        return new Trajectory(trajectory.getStates().stream()
                .map(state -> new Trajectory.State(
                        state.timeSeconds,
                        state.linearVelocity,
                        Arrays.stream(state.feedforwards.accelerations())
                                .reduce(LinearAcceleration::plus)
                                .orElseGet(() -> LinearAcceleration.ofBaseUnits(0.0, Units.MetersPerSecondPerSecond))
                                .in(Units.MetersPerSecondPerSecond),
                        state.pose,
                        0.0))
                .toList());
    }
}
