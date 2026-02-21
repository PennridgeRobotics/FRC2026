// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package org.pennridge.robotics.frc.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.jspecify.annotations.NullMarked;
import org.pennridge.robotics.frc.util.enums.Constants.FuelConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

@NullMarked
public class FuelSubsystem extends SubsystemBase {

    private final SimpleMotorFeedforward feederFeedForward = new SimpleMotorFeedforward(0.0, 0.0);
    private final SimpleMotorFeedforward launcherFeedForward = new SimpleMotorFeedforward(0.0, 0.0);
    private final PIDController feederPIDController = new PIDController(0.0, 0.0, 0.0);
    private final PIDController launcherPIDController = new PIDController(0.0, 0.0, 0.0);

    private FuelAction currentState;
    private final Trigger launching;
    private final Trigger ejecting;
    private final Trigger intaking;
    private final Trigger spinningUp;

    /** Creates a new FeederSubsystem. */
    public FuelSubsystem() {
        setDefaultCommand(run(() -> {
            currentState = FuelAction.NONE;
            feederMotorController.setDutyCycle(0);
            launcherMotorController.setDutyCycle(0);
        }));
        currentState = FuelAction.NONE;
        launching = new Trigger(() -> currentState == FuelAction.LAUNCHING);
        ejecting = new Trigger(() -> currentState == FuelAction.EJECTING);
        intaking = new Trigger(() -> currentState == FuelAction.INTAKING);
        spinningUp = new Trigger(() -> currentState == FuelAction.SPINNING_UP);

        setupSmartDashboard();
    }

    private final SmartMotorControllerConfig feederSMCConfig = new SmartMotorControllerConfig(this)
            .withFeedforward(feederFeedForward)
            .withClosedLoopController(feederPIDController)
            // Telemetry name and verbosity level
            .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH)
            // Gearing from the motor rotor to the final shaft.
            // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which
            // corresponds to the gearbox attached to your motor.
            // You could also use .withGearing(12) which does the same thing.
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            // Motor properties to prevent over currenting.
            .withMotorInverted(false)
            .withVoltageCompensation(Volts.of(12))
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40));

    private final SmartMotorControllerConfig launcherSMCConfig = new SmartMotorControllerConfig(this)
            .withFeedforward(launcherFeedForward)
            .withClosedLoopController(launcherPIDController)
            // Telemetry name and verbosity level
            .withTelemetry("LauncherMotor", TelemetryVerbosity.HIGH)
            // Gearing from the motor rotor to the final shaft.
            // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which
            // corresponds to the gearbox attached to your motor.
            // You could also use .withGearing(12) which does the same thing.
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            // Motor properties to prevent over currenting.
            .withMotorInverted(false)
            .withVoltageCompensation(Volts.of(12))
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40));

    // Vendor motor controller object

    private final SparkMax feeder = new SparkMax(FuelConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax launcher = new SparkMax(FuelConstants.INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    // Create our SmartMotorController from our Spark and config with the NEO.
    private final SmartMotorController feederMotorController =
            new SparkWrapper(feeder, DCMotor.getNEO(1), feederSMCConfig);
    private final FlyWheelConfig feederConfig = new FlyWheelConfig(feederMotorController)
            .withDiameter(Inches.of(4))
            .withMass(Pounds.of(1))
            .withUpperSoftLimit(RPM.of(1000))
            .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController launcherMotorController =
            new SparkWrapper(launcher, DCMotor.getNEO(1), launcherSMCConfig);
    private final FlyWheelConfig launcherConfig = new FlyWheelConfig(launcherMotorController)
            .withDiameter(Inches.of(4))
            .withMass(Pounds.of(1))
            .withUpperSoftLimit(RPM.of(1000))
            .withTelemetry("LauncherMotor", TelemetryVerbosity.HIGH);
    private final FlyWheel feederMotor = new FlyWheel(feederConfig);
    private final FlyWheel launcherMotor = new FlyWheel(launcherConfig);

    private void setupSmartDashboard() {
        SmartDashboard.putNumber("Intaking feeder roller value", FuelConstants.INDEXER_INTAKING_PERCENT);
        SmartDashboard.putNumber("Intaking intake roller value", FuelConstants.INTAKE_INTAKING_PERCENT);
        SmartDashboard.putNumber("Launching feeder roller value", FuelConstants.INDEXER_LAUNCHING_PERCENT);
        SmartDashboard.putNumber("Launching launcher roller value", FuelConstants.LAUNCHING_LAUNCHER_PERCENT);

        SmartDashboard.putData("Feeder", (builder) -> {
            builder.addDoubleProperty("Velocity", () -> feederMotor.getSpeed().in(DegreesPerSecond), null);
            builder.addDoubleProperty(
                    "DutyCycle", feederMotorController::getDutyCycle, feederMotorController::setDutyCycle);
        });
        SmartDashboard.putData("Feeder", (builder) -> {
            builder.addDoubleProperty("Velocity", () -> launcherMotor.getSpeed().in(DegreesPerSecond), null);
            builder.addDoubleProperty(
                    "DutyCycle", launcherMotorController::getDutyCycle, launcherMotorController::setDutyCycle);
        });
    }

    public Command eject() {
        return run(() -> {
            currentState = FuelAction.EJECTING;
            launcherMotorController.setDutyCycle(
                    -1 * SmartDashboard.getNumber("Intaking intake roller value", FuelConstants.INTAKE_EJECT_PERCENT));
            feederMotorController.setDutyCycle(
                    SmartDashboard.getNumber("Intaking intake roller value", FuelConstants.INDEXER_LAUNCHING_PERCENT));
        });
    }

    public Command intake() {
        return run(() -> {
            currentState = FuelAction.INTAKING;
            launcherMotorController.setDutyCycle(
                    SmartDashboard.getNumber("Intaking intake roller value", FuelConstants.INTAKE_INTAKING_PERCENT));
            feederMotorController.setDutyCycle(
                    SmartDashboard.getNumber("Intaking feeder roller value", FuelConstants.INDEXER_INTAKING_PERCENT));
        });
    }

    public Command launch() {
        return run(() -> {
            currentState = FuelAction.LAUNCHING;
            launcherMotorController.setDutyCycle(SmartDashboard.getNumber(
                    "Launching launcher roller value", FuelConstants.LAUNCHING_LAUNCHER_PERCENT));
            feederMotorController.setDutyCycle(
                    SmartDashboard.getNumber("Launching feeder roller value", FuelConstants.INDEXER_LAUNCHING_PERCENT));
        });
    }

    public Command spinUp() {
        return run(() -> {
            currentState = FuelAction.SPINNING_UP;
            launcherMotorController.setDutyCycle(SmartDashboard.getNumber(
                    "Launching launcher roller value", FuelConstants.LAUNCHING_LAUNCHER_PERCENT));
            feederMotorController.setDutyCycle(SmartDashboard.getNumber(
                    "Launching spin-up feeder value", FuelConstants.INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT));
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
        feederMotor.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        feederMotor.simIterate();
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
