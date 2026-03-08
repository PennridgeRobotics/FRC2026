package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.dashboard.LoggedNetworkDouble;
import frc.robot.util.dashboard.MultiMotorInfoSendable;
import frc.robot.util.dashboard.PIDSendable;
import frc.robot.util.enums.Constants.ClimberConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.jspecify.annotations.NullMarked;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

@NullMarked
public class ClimberSubsystem extends SubsystemBase {
    private final SmartMotorController motorController;
    private final Arm climber;

    private boolean isClimbing = false;
    private final Trigger climbingTrigger = new Trigger(() -> isClimbing);
    private final Trigger isClimbed;
    private final Trigger isArmed;
    private final DoubleSupplier climbValue =
            new LoggedNetworkDouble("Climber/Climb Value", ClimberConstants.CLIMB_VALUE);
    private final DoubleSupplier climbFastValue =
            new LoggedNetworkDouble("Climber/Climb Fast Value", ClimberConstants.CLIMB_FAST_VALUE);
    private final DoubleSupplier lowerValue =
            new LoggedNetworkDouble("Climber/Lower Value", ClimberConstants.LOWER_VALUE);
    private final DoubleSupplier lowerFastValue =
            new LoggedNetworkDouble("Climber/Lower Fast Value", ClimberConstants.LOWER_FAST_VALUE);

    public ClimberSubsystem(MultiMotorInfoSendable motorInfo) {
        final var motorConfig = new SmartMotorControllerConfig()
                .withSubsystem(this)
                .withMotorInverted(ClimberConstants.CLIMBER_INVERTED)
                .withIdleMode(ClimberConstants.IDLE_MODE)
                .withControlMode(ControlMode.OPEN_LOOP)
                .withGearing(ClimberConstants.CLIMBER_GEARING)
                .withStatorCurrentLimit(ClimberConstants.CURRENT_LIMIT)
                .withOpenLoopRampRate(ClimberConstants.RAMP_RATE)
                .withTelemetry("ClimberMotor", TelemetryVerbosity.HIGH)
                .withSoftLimit(Degrees.of(-360), Degrees.of(360)) // no soft limit because no absolute encoder
                .withVoltageCompensation(ClimberConstants.VOLTAGE_COMPENSATION);
        final var sparkMaxMotor = new SparkMax(ClimberConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
        motorController = new SparkWrapper(sparkMaxMotor, DCMotor.getNEO(1), motorConfig);
        climber = new Arm(new ArmConfig(motorController)
                .withStartingPosition(ClimberConstants.VERTICAL_ANGLE)
                .withTelemetry("ClimberArm", TelemetryVerbosity.HIGH));

        isClimbed = new Trigger(
                () -> climber.getAngle().gte(ClimberConstants.CLIMBED_ANGLE.minus(ClimberConstants.TOLERANCE_ANGLE)));
        isArmed = new Trigger(
                () -> climber.getAngle().lte(ClimberConstants.ARMED_ANGLE.plus(ClimberConstants.TOLERANCE_ANGLE)));

        motorInfo.addMotor(sparkMaxMotor, "Climber");

        setupSmartDashboard();

        setDefaultCommand(startRun(() -> System.out.println("CLIMBER VALUE: 0.0"), () -> climber.getMotor()
                .setDutyCycle(0.0)));
    }

    private void setupSmartDashboard() {
        SmartDashboard.putData(
                "Climber PID & FF",
                new PIDSendable(motorController, PIDSendable.Type.PID | PIDSendable.Type.ROTARY_FF));
        SmartDashboard.putData("Climbing Subsystem", (builder) -> {
            builder.addBooleanProperty("Climbing", () -> isClimbing, (v) -> isClimbing = v);
            builder.addDoubleProperty("Angle", () -> climber.getAngle().in(Degrees), v -> climber.getMotor()
                    .setEncoderPosition(Degrees.of(v)));
            builder.addDoubleProperty(
                    "Angular Velocity",
                    () -> climber.getMotor().getMechanismVelocity().in(DegreesPerSecond),
                    null);
        });
    }

    public Command climbCommand(BooleanSupplier autoStop, BooleanSupplier fast) {
        return Commands.sequence(
                        Commands.runOnce(() -> isClimbing = true),
                        Commands.either(
                                climber.run(ClimberConstants.CLIMBED_ANGLE),
                                setDutyCycle(() ->
                                        fast.getAsBoolean() ? climbFastValue.getAsDouble() : climbValue.getAsDouble()),
                                () -> false))
                .until(isClimbed.and(autoStop));
    }

    public Command armCommand(BooleanSupplier autoStop, BooleanSupplier fast) {
        return Commands.either(
                        climber.run(ClimberConstants.ARMED_ANGLE),
                        setDutyCycle(
                                () -> fast.getAsBoolean() ? lowerFastValue.getAsDouble() : lowerValue.getAsDouble()),
                        () -> false)
                .until(isArmed.and(autoStop));
    }

    private Command setDutyCycle(DoubleSupplier dutyCycleSupplier) {
        return startRun(
                        () -> System.out.println("CLIMBER VALUE: " + dutyCycleSupplier.getAsDouble()),
                        () -> climber.getMotor().setDutyCycle(dutyCycleSupplier.getAsDouble()))
                .finallyDo(() -> motorController.setDutyCycle(0));
    }

    public Command setClimberEncoderToVertical() {
        return Commands.runOnce(() -> climber.getMotor().setEncoderPosition(ClimberConstants.VERTICAL_ANGLE));
    }

    public Command findLimit() {
        final var currentDebouncer = new Debouncer(0.1);
        final var runVolts = Volts.of(-1.0);
        final var currentThreshold = Amps.of(0); // change this
        final var velocityThreshold = DegreesPerSecond.of(2);
        return startRun(() -> isClimbing = false, () -> motorController.setVoltage(runVolts))
                .until(() -> currentDebouncer.calculate(
                        motorController.getStatorCurrent().gte(currentThreshold)
                                && motorController.getMechanismVelocity().abs(DegreesPerSecond)
                                        <= velocityThreshold.in(DegreesPerSecond)))
                .finallyDo(() -> {
                    motorController.setVoltage(Volts.zero());
                    motorController.setEncoderPosition(ClimberConstants.MINIMUM_ANGLE);
                });
    }

    @Override
    public void periodic() {
        climber.updateTelemetry();
        System.out.println("Climber output: " + climber.getMotor().getDutyCycle());
    }

    public Command sysId() {
        return climber.sysId(Volts.of(3), Volts.of(3).per(Second), Seconds.of(30));
    }

    @Override
    public void simulationPeriodic() {
        climber.simIterate();
    }

    public Trigger getClimbingTrigger() {
        return climbingTrigger;
    }
}
