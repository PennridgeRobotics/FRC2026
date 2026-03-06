package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.dashboard.LoggedNetworkUnit;
import frc.robot.util.dashboard.MultiMotorInfoSendable;
import frc.robot.util.dashboard.PIDSendable;
import frc.robot.util.enums.Constants.ClimberConstants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
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
    private boolean closedLoopEnabled = true;
    private final Supplier<Voltage> climbVoltage =
            new LoggedNetworkUnit<>("Climber/Climb Voltage", ClimberConstants.CLIMB_VOLTAGE);
    private final Supplier<Voltage> lowerVoltage =
            new LoggedNetworkUnit<>("Climber/Lower Voltage", ClimberConstants.LOWER_VOLTAGE);

    public ClimberSubsystem(MultiMotorInfoSendable motorInfo) {
        final var motorConfig = new SmartMotorControllerConfig()
                .withSubsystem(this)
                .withMotorInverted(ClimberConstants.CLIMBER_INVERTED)
                .withIdleMode(ClimberConstants.IDLE_MODE)
                .withControlMode(closedLoopEnabled ? ControlMode.CLOSED_LOOP : ControlMode.OPEN_LOOP)
                .withGearing(ClimberConstants.CLIMBER_GEARING)
                .withStatorCurrentLimit(ClimberConstants.CURRENT_LIMIT)
                .withOpenLoopRampRate(ClimberConstants.RAMP_RATE)
                .withTelemetry("ClimberMotor", TelemetryVerbosity.HIGH)
                // .withSoftLimit(ClimberConstants.MINIMUM_ANGLE, ClimberConstants.MAXIMUM_ANGLE)
                .withClosedLoopController(new PIDController(0, 0, 0))
                .withFeedforward(new ArmFeedforward(0, 0, 0, 0));
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

        /*setDefaultCommand(run(() -> {
            if (closedLoopEnabled) return;
            climber.getMotor().setVoltage(Volts.of(0.0));
        }));*/
    }

    private void setupSmartDashboard() {
        SmartDashboard.putData(
                "Climber PID & FF",
                new PIDSendable(motorController, PIDSendable.Type.PID | PIDSendable.Type.ROTARY_FF));
        SmartDashboard.putData("Climbing Subsystem", (builder) -> {
            builder.addBooleanProperty("Climbing", () -> isClimbing, (v) -> isClimbing = v);
            builder.addBooleanProperty("Closed loop enabled", () -> closedLoopEnabled, (v) -> {
                if (v) motorController.startClosedLoopController();
                else motorController.stopClosedLoopController();
                closedLoopEnabled = v;
            });
            builder.addDoubleProperty("Angle", () -> climber.getAngle().in(Degrees), null);
            builder.addDoubleProperty(
                    "Angular Velocity",
                    () -> climber.getMotor().getMechanismVelocity().in(DegreesPerSecond),
                    null);
        });
    }

    public Command climbCommand(BooleanSupplier autoStop) {
        return Commands.sequence(
                        Commands.runOnce(() -> isClimbing = true),
                        Commands.either(
                                climber.run(ClimberConstants.CLIMBED_ANGLE),
                                setVoltage(climbVoltage),
                                () -> closedLoopEnabled))
                .until(isClimbed.and(autoStop));
    }

    public Command armCommand(BooleanSupplier autoStop) {
        return Commands.either(
                        climber.run(ClimberConstants.ARMED_ANGLE), setVoltage(lowerVoltage), () -> closedLoopEnabled)
                .until(isArmed.and(autoStop));
    }

    private Command setVoltage(Supplier<Voltage> voltageSupplier) {
        return startRun(
                        motorController::stopClosedLoopController,
                        () -> motorController.setVoltage(voltageSupplier.get()))
                .finallyDo(() -> {
                    if (closedLoopEnabled) motorController.startClosedLoopController();
                });
    }

    public Command setClimberEncoderToVertical() {
        return Commands.runOnce(() -> {
            System.out.println("TEST");
            climber.getMotor().setEncoderPosition(ClimberConstants.VERTICAL_ANGLE);
        });
    }

    public Command findLimit() {
        final var currentDebouncer = new Debouncer(0.1);
        final var runVolts = Volts.of(-1.0);
        final var currentThreshold = Amps.of(0); // change this
        final var velocityThreshold = DegreesPerSecond.of(2);
        return startRun(
                        () -> {
                            isClimbing = false;
                            motorController.stopClosedLoopController();
                        },
                        () -> motorController.setVoltage(runVolts))
                .until(() -> currentDebouncer.calculate(
                        motorController.getStatorCurrent().gte(currentThreshold)
                                && motorController.getMechanismVelocity().abs(DegreesPerSecond)
                                        <= velocityThreshold.in(DegreesPerSecond)))
                .finallyDo(() -> {
                    motorController.setVoltage(Volts.zero());
                    motorController.setEncoderPosition(ClimberConstants.MINIMUM_ANGLE);
                    if (closedLoopEnabled) {
                        motorController.startClosedLoopController();
                    }
                });
    }

    @Override
    public void periodic() {
        climber.updateTelemetry();
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
