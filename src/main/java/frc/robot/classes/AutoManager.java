package frc.robot.classes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoManager {
    private final SwerveSubsystem swerveDrive;
    private final FollowPath.Builder pathBuilder;

    public AutoManager(SwerveSubsystem swerveDrive, FollowPath.Builder pathBuilder) {
        this.swerveDrive = swerveDrive;
        this.pathBuilder = pathBuilder;
    }

    public Command followPath(String pathName) {
        Path path = new Path(pathName);
        return followPath(path);
    }

    public FollowPath followPath(Path path) {
        return pathBuilder.build(path);
    }
}
