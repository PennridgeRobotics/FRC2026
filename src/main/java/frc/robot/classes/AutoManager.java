package frc.robot.classes;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoManager {
    private final SwerveSubsystem swerveDrive;
    private final FollowPath.Builder pathBuilder;
    private final FuelSubsystem fuelSubsystem;

    public AutoManager(SwerveSubsystem swerveDrive, FollowPath.Builder pathBuilder, FuelSubsystem fuelSubsystem) {
        this.swerveDrive = swerveDrive;
        this.fuelSubsystem = fuelSubsystem;
        this.pathBuilder = pathBuilder;
    }

    public FollowPath followPath(String pathName) {
        Path path = new Path(pathName);
        return followPath(path);
    }

    public FollowPath followPath(Path path) {
        return pathBuilder.build(path);
    }

    public Command fieldAutos() {
        return Commands.sequence(shootFromStart()).withTimeout(Seconds.of(20));
    }

    public Command shootFromStart() {
        return Commands.sequence(followPath("shoot_from_start"), fuelSubsystem.windUp());
    }
}
