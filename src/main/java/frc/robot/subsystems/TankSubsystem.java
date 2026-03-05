package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class TankSubsystem extends SubsystemBase {
    private final SparkMax leftLeader;
    // private final SparkMax leftFollower;
    private final SparkMax rightLeader;
    // private final SparkMax rightFollower;

    // private final DifferentialDrive drive;

    public TankSubsystem() {
        // create brushed motors for drive
        leftLeader = new SparkMax(5, MotorType.kBrushless);
        rightLeader = new SparkMax(6, MotorType.kBrushless);
        // leftFollower = new SparkMax(7, MotorType.kBrushless); // BL
        // rightLeader = new SparkMax(3, MotorType.kBrushless); // FR
        // rightFollower = new SparkMax(5, MotorType.kBrushless); // BR

        // set up differential drive class
        // drive = new DifferentialDrive(leftLeader, rightLeader);

        // Set can timeout. Because this project only sets parameters once on
        // construction, the timeout can be long without blocking robot operation. Code
        // which sets or gets parameters during operation may need a shorter timeout.
        leftLeader.setCANTimeout(250);
        rightLeader.setCANTimeout(250);
        /* rightLeader.setCANTimeout(250);
        leftFollower.setCANTimeout(250);
        rightFollower.setCANTimeout(250);*/

        // Create the configuration to apply to motors. Voltage compensation
        // helps the robot perform more similarly on different
        // battery voltages (at the cost of a little bit of top speed on a fully charged
        // battery). The current limit helps prevent tripping
        // breakers.
        SparkMaxConfig config = new SparkMaxConfig();
        config.voltageCompensation(12);
        config.smartCurrentLimit(40);

        // Set configuration to follow each leader and then apply it to corresponding
        // follower. Resetting in case a new controller is swapped
        // in and persisting in case of a controller reset due to breaker trip
        /*config.follow(leftLeader);
        leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.follow(rightLeader);
        rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Remove following, then apply config to right leader
        config.disableFollowerMode();
        rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // Set config to inverted and then apply to left leader. Set Left side inverted
        // so that positive values drive both sides forward
        config.inverted(true);*/
        leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {}

    // Command factory to create command to drive the robot with joystick inputs.
    public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
        // return this.run(() -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
        return this.runEnd(
                () -> {
                    leftLeader.set(xSpeed.getAsDouble());
                    rightLeader.set(zRotation.getAsDouble());
                },
                () -> {
                    leftLeader.set(0);
                    rightLeader.set(0);
                });
    }
}
