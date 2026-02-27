// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.ShooterCalculator;
import frc.robot.util.dashboard.PIDSendable;
import frc.robot.util.enums.Constants.FuelConstants;
import org.jspecify.annotations.NullMarked;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

@NullMarked
public class FuelSubsystem extends SubsystemBase {

    private FuelAction currentState;
    private final ShooterCalculator shooterCalculator;

    private final Trigger launching;
    private final Trigger ejecting;
    private final Trigger intaking;
    private final Trigger spinningUp;

    private final SmartMotorController intakeLauncherMotorController;
    private final SmartMotorController indexerMotorController;

    private final FlyWheel intakeLauncher;
    private final FlyWheel indexer;

    public FuelSubsystem(ShooterCalculator shooterCalculator) {
        this.shooterCalculator = shooterCalculator;

        final var baseIntakeLauncherSMCConfig = new SmartMotorControllerConfig(this)
                .withGearing(FuelConstants.INTAKE_LAUNCHER_GEARING)
                .withOpenLoopRampRate(FuelConstants.INTAKE_LAUNCHER_RAMP_RATE)
                .withMotorInverted(FuelConstants.INTAKE_LAUNCHER_INVERTED)
                .withVoltageCompensation(FuelConstants.INTAKE_LAUNCHER_VOLTAGE_COMP)
                .withIdleMode(FuelConstants.INTAKE_LAUNCHER_MOTOR_MODE)
                .withStatorCurrentLimit(FuelConstants.INTAKE_LAUNCHER_CURRENT_LIMIT);
        final var intakeLauncherLeftSMCConfig = baseIntakeLauncherSMCConfig
                .clone()
                .withFeedforward(new SimpleMotorFeedforward(0.0, 0.0))
                .withClosedLoopController(new PIDController(0.0, 0.0, 0.0))
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withTelemetry("LauncherMotor", TelemetryVerbosity.HIGH);
        final var indexerSMCConfig = new SmartMotorControllerConfig(this)
                .withFeedforward(new SimpleMotorFeedforward(0.0, 0.0))
                .withClosedLoopController(new PIDController(0.0, 0.0, 0.0))
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH)
                .withGearing(FuelConstants.INDEXER_GEARING)
                .withOpenLoopRampRate(FuelConstants.INDEXER_RAMP_RATE)
                .withMotorInverted(FuelConstants.INDEXER_INVERTED)
                .withVoltageCompensation(FuelConstants.INDEXER_VOLTAGE_COMP)
                .withIdleMode(FuelConstants.INDEXER_MOTOR_MODE)
                .withStatorCurrentLimit(FuelConstants.INDEXER_CURRENT_LIMIT)
                .withFollowers();

        final var intakeLauncherLeftSparkMax =
                new SparkMax(FuelConstants.INTAKE_LAUNCHER_LEFT_MOTOR_ID, MotorType.kBrushless);
        final var intakeLauncherRightSparkMax =
                new SparkMax(FuelConstants.INTAKE_LAUNCHER_RIGHT_MOTOR_ID, MotorType.kBrushless);
        final var indexerSparkMax = new SparkMax(FuelConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);

        // apply config
        new SparkWrapper(intakeLauncherRightSparkMax, DCMotor.getNEO(1), baseIntakeLauncherSMCConfig);

        intakeLauncherLeftSMCConfig.withFollowers(Pair.of(intakeLauncherRightSparkMax, true));
        intakeLauncherMotorController =
                new SparkWrapper(intakeLauncherLeftSparkMax, DCMotor.getNEO(2), intakeLauncherLeftSMCConfig);
        indexerMotorController = new SparkWrapper(indexerSparkMax, DCMotor.getNEO(1), indexerSMCConfig);

        intakeLauncher = new FlyWheel(new FlyWheelConfig(intakeLauncherMotorController)
                .withDiameter(Inches.of(4))
                .withTelemetry("LauncherMotor", TelemetryVerbosity.HIGH));
        indexer = new FlyWheel(new FlyWheelConfig(indexerMotorController)
                .withDiameter(Inches.of(4))
                .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH));

        setDefaultCommand(run(() -> {
            currentState = FuelAction.NONE;
            intakeLauncherMotorController.setDutyCycle(0);
            indexerMotorController.setDutyCycle(0);
        }));
        currentState = FuelAction.NONE;
        launching = new Trigger(() -> currentState == FuelAction.LAUNCHING);
        ejecting = new Trigger(() -> currentState == FuelAction.EJECTING);
        intaking = new Trigger(() -> currentState == FuelAction.INTAKING);
        spinningUp = new Trigger(() -> currentState == FuelAction.SPINNING_UP);

        setupSmartDashboard();
    }

    private void setupSmartDashboard() {
        SmartDashboard.putNumber("Intaking indexer roller value", FuelConstants.INDEXER_INTAKING_PERCENT);
        SmartDashboard.putNumber("Intaking intake roller value", FuelConstants.INTAKE_INTAKING_PERCENT);
        SmartDashboard.putNumber("Launching indexer roller value", FuelConstants.INDEXER_LAUNCHING_PERCENT);
        SmartDashboard.putNumber("Launching launcher roller value", FuelConstants.LAUNCHING_LAUNCHER_PERCENT);
        SmartDashboard.putData("Intake/Launcher", (builder) -> {
            builder.addDoubleProperty(
                    "Velocity", () -> intakeLauncher.getSpeed().in(RotationsPerSecond), null);
            builder.addDoubleProperty(
                    "DutyCycle",
                    intakeLauncherMotorController::getDutyCycle,
                    intakeLauncherMotorController::setDutyCycle);
        });
        SmartDashboard.putData("Indexer", (builder) -> {
            builder.addDoubleProperty("Velocity", () -> indexer.getSpeed().in(RotationsPerSecond), null);
            builder.addDoubleProperty(
                    "DutyCycle", indexerMotorController::getDutyCycle, indexerMotorController::setDutyCycle);
        });
        SmartDashboard.putData(
                "Intake/Launcher PID",
                new PIDSendable(intakeLauncherMotorController, PIDSendable.Type.PID | PIDSendable.Type.BASE_FF));
        SmartDashboard.putData(
                "Indexer PID",
                new PIDSendable(indexerMotorController, PIDSendable.Type.PID | PIDSendable.Type.BASE_FF));
        SmartDashboard.putData(
                "Fuel Subsystem",
                (builder) -> builder.addStringProperty("Current State", () -> currentState.toString(), null));
    }

    public Command eject() {
        return run(() -> {
            currentState = FuelAction.EJECTING;
            intakeLauncherMotorController.setDutyCycle(
                    -1 * SmartDashboard.getNumber("Intaking intake roller value", FuelConstants.INTAKE_EJECT_PERCENT));
            indexerMotorController.setDutyCycle(
                    SmartDashboard.getNumber("Intaking intake roller value", FuelConstants.INDEXER_LAUNCHING_PERCENT));
        });
    }

    public Command intake() {
        return run(() -> {
            currentState = FuelAction.INTAKING;
            intakeLauncherMotorController.setDutyCycle(
                    SmartDashboard.getNumber("Intaking intake roller value", FuelConstants.INTAKE_INTAKING_PERCENT));
            indexerMotorController.setDutyCycle(
                    SmartDashboard.getNumber("Intaking indexer roller value", FuelConstants.INDEXER_INTAKING_PERCENT));
        });
    }

    public Command launch() {
        return run(() -> {
            currentState = FuelAction.LAUNCHING;
            intakeLauncherMotorController.setDutyCycle(SmartDashboard.getNumber(
                    "Launching launcher roller value", FuelConstants.LAUNCHING_LAUNCHER_PERCENT));
            indexerMotorController.setDutyCycle(SmartDashboard.getNumber(
                    "Launching indexer roller value", FuelConstants.INDEXER_LAUNCHING_PERCENT));
        });
    }

    public Command spinUp() {
        return run(() -> {
            currentState = FuelAction.SPINNING_UP;
            intakeLauncherMotorController.setDutyCycle(SmartDashboard.getNumber(
                    "Launching launcher roller value", FuelConstants.LAUNCHING_LAUNCHER_PERCENT));
            indexerMotorController.setDutyCycle(SmartDashboard.getNumber(
                    "Launching spin-up indexer value", FuelConstants.INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT));
        });
    }

    public Command launchSequence() {
        return Commands.sequence(spinUp().withTimeout(FuelConstants.SPIN_UP_SECONDS), launch());
    }

    public Trigger isLaunchingTrigger() {
        return launching;
    }

    public Trigger isIntakingTrigger() {
        return intaking;
    }

    public Trigger isEjectingTrigger() {
        return ejecting;
    }

    public Trigger isSpinningUpTrigger() {
        return spinningUp;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        indexer.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        indexer.simIterate();
    }

    // COMMANDS:

    /*
     * EJECT
     * INTAKE
     * LAUNCH
     * SPIN UP
     * LAUNCH SEQUENCE
     */

    enum FuelAction {
        EJECTING,
        LAUNCHING,
        INTAKING,
        SPINNING_UP,
        NONE
    };
}
