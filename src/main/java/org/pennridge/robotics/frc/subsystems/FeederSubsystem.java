// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package org.pennridge.robotics.frc.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jspecify.annotations.NullMarked;
import org.pennridge.robotics.frc.util.enums.Constants.FuelConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

@NullMarked
public class FeederSubsystem extends SubsystemBase {

    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.OPEN_LOOP)
            // Telemetry name and verbosity level
            .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH)
            // Gearing from the motor rotor to final shaft.
            // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which
            // corresponds to the gearbox attached to your motor.
            // You could also use .withGearing(12) which does the same thing.
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            // Motor properties to prevent over currenting.
            .withMotorInverted(false)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40));

    // Vendor motor controller object

    private SparkMax spark = new SparkMax(FuelConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
    // Create our SmartMotorController from our Spark and config with the NEO.
    private SmartMotorController motorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);
    private final FlyWheelConfig shooterConfig = new FlyWheelConfig(motorController)
            .withDiameter(Inches.of(4))
            .withMass(Pounds.of(1))
            .withUpperSoftLimit(RPM.of(1000))
            .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH);

    private FlyWheel shooter = new FlyWheel(shooterConfig);

    /**
     * Gets the current velocity of the shooter.
     *
     * @return Shooter velocity.
     */
    public AngularVelocity getVelocity() {
        return shooter.getSpeed();
    }

    /**
     * Set the shooter velocity.
     *
     * @param speed Speed to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command setVelocity(AngularVelocity speed) {
        return shooter.setSpeed(speed);
    }

    /**
     * Set the dutycycle of the shooter.
     *
     * @param dutyCycle DutyCycle to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command set(double dutyCycle) {
        return shooter.set(dutyCycle);
    }

    /** Creates a new FeederSubsystem. */
    public FeederSubsystem() {}

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(() -> {
            /* one-time action goes here */
        });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        shooter.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        shooter.simIterate();
    }
}
