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
// COMMANDS:
/*
 * EJECT
 * INTAKE
 * LAUNCH
 * SPIN UP
 * LAUNCH SEQUENCE
*/


enum Action {
    EJECTING,
    LAUNCHING,
    INTAKING,
    SPINNING_UP,
    NONE
};

@NullMarked
public class FuelSubsystem extends SubsystemBase {
    
    private SimpleMotorFeedforward feederFeedForward = new SimpleMotorFeedforward(0.0, 0.0);
    private SimpleMotorFeedforward launcherFeedForward = new SimpleMotorFeedforward(0.0, 0.0);
    private PIDController feederPIDController = new PIDController(0.0, 0.0, 0.0);
    private PIDController launcherPIDController = new PIDController(0.0, 0.0, 0.0);

    private Action state;
    private final Trigger launching;
    private final Trigger ejecting;
    private final Trigger intaking;
    private final Trigger spinning_up;



    /** Creates a new FeederSubsystem. */
    public FuelSubsystem() {
        setDefaultCommand(run(() -> {
            feederMotorController.setDutyCycle(0);
            launcherMotorController.setDutyCycle(0);
        }));
        state = Action.NONE;
        launching = new Trigger(() -> state == Action.LAUNCHING);
        ejecting = new Trigger(() -> state == Action.EJECTING);
        intaking = new Trigger(() -> state == Action.INTAKING);
        spinning_up = new Trigger(() -> state == Action.SPINNING_UP);

        setupSmartDashboard();
    }

    private SmartMotorControllerConfig feederSMCConfig = new SmartMotorControllerConfig(this)
            .withFeedforward(feederFeedForward)
            .withClosedLoopController(feederPIDController)
            // Telemetry name and verbosity level
            .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH)
            // Gearing from the motor rotor to final shaft.
            // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which
            // corresponds to the gearbox attached to your motor.
            // You could also use .withGearing(12) which does the same thing.
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            // Motor properties to prevent over currenting.
            .withMotorInverted(false)
            .withVoltageCompensation(Volts.of(12))
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40));

    private SmartMotorControllerConfig launcherSMCConfig = new SmartMotorControllerConfig(this)
            .withFeedforward(launcherFeedForward)
            .withClosedLoopController(launcherPIDController)
            // Telemetry name and verbosity level
            .withTelemetry("LauncherMotor", TelemetryVerbosity.HIGH)
            // Gearing from the motor rotor to final shaft.
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

    private SparkMax feeder = new SparkMax(FuelConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
    private SparkMax launcher = new SparkMax(FuelConstants.INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    // Create our SmartMotorController from our Spark and config with the NEO.
    private SmartMotorController feederMotorController = new SparkWrapper(feeder, DCMotor.getNEO(1), feederSMCConfig);
    private final FlyWheelConfig feederConfig = new FlyWheelConfig(feederMotorController)
            .withDiameter(Inches.of(4))
            .withMass(Pounds.of(1))
            .withUpperSoftLimit(RPM.of(1000))
            .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH);

    private SmartMotorController launcherMotorController =
            new SparkWrapper(launcher, DCMotor.getNEO(1), launcherSMCConfig);
    private final FlyWheelConfig launcherConfig = new FlyWheelConfig(launcherMotorController)
            .withDiameter(Inches.of(4))
            .withMass(Pounds.of(1))
            .withUpperSoftLimit(RPM.of(1000))
            .withTelemetry("LauncherMotor", TelemetryVerbosity.HIGH);
    private FlyWheel feederMotor = new FlyWheel(feederConfig);
    private FlyWheel launcherMotor = new FlyWheel(launcherConfig);


    private void setupSmartDashboard() {
        SmartDashboard.putNumber("Intaking feeder roller value", FuelConstants.INDEXER_INTAKING_PERCENT);
        SmartDashboard.putNumber("Intaking intake roller value", FuelConstants.INTAKE_INTAKING_PERCENT);
        SmartDashboard.putNumber("Launching feeder roller value", FuelConstants.INDEXER_LAUNCHING_PERCENT);
        SmartDashboard.putNumber("Launching launcher roller value", FuelConstants.LAUNCHING_LAUNCHER_PERCENT);

        SmartDashboard.putData("Feeder", (builder) -> {
            builder.addDoubleProperty("Velocity", () -> feederMotor.getSpeed().in(DegreesPerSecond), null);
            builder.addDoubleProperty(
                    "DutyCycle",
                    () -> feederMotorController.getDutyCycle(),
                    (f) -> feederMotorController.setDutyCycle(f));
        });
        SmartDashboard.putData("Feeder", (builder) -> {
            builder.addDoubleProperty("Velocity", () -> launcherMotor.getSpeed().in(DegreesPerSecond), null);
            builder.addDoubleProperty(
                    "DutyCycle",
                    () -> launcherMotorController.getDutyCycle(),
                    (f) -> launcherMotorController.setDutyCycle(f));
        });
    }


    public Command eject() {
        return run(() -> {
            state = Action.EJECTING;
            launcherMotorController.setDutyCycle(
                    -1 * SmartDashboard.getNumber("Intaking intake roller value", FuelConstants.INTAKE_EJECT_PERCENT));
            feederMotorController.setDutyCycle(
                    SmartDashboard.getNumber("Intaking intake roller value", FuelConstants.INDEXER_LAUNCHING_PERCENT));
        });
    }

    public Command intake() {
        return run(() -> {
            state = Action.INTAKING;
            launcherMotorController.setDutyCycle(
                    SmartDashboard.getNumber("Intaking intake roller value", FuelConstants.INTAKE_INTAKING_PERCENT));
            feederMotorController.setDutyCycle(
                    SmartDashboard.getNumber("Intaking feeder roller value", FuelConstants.INDEXER_INTAKING_PERCENT));
        });
    }

    public Command launch() {
        return run(() -> {
            state = Action.LAUNCHING;
            launcherMotorController.setDutyCycle(SmartDashboard.getNumber(
                    "Launching launcher roller value", FuelConstants.LAUNCHING_LAUNCHER_PERCENT));
            feederMotorController.setDutyCycle(
                    SmartDashboard.getNumber("Launching feeder roller value", FuelConstants.INDEXER_LAUNCHING_PERCENT));
        });
    }

    public Command spinUp() {
        return run(() -> {
            state = Action.SPINNING_UP;
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
        return spinning_up;
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
}
