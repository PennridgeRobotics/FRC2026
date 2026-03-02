package frc.robot.util.dashboard;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.Comparator;
import java.util.NavigableMap;
import java.util.TreeMap;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class MultiMotorInfoSendable implements Sendable {
    private final NavigableMap<SparkMax, String> motors =
            new TreeMap<>(Comparator.comparingInt(SparkLowLevel::getDeviceId));

    @Override
    public void initSendable(SendableBuilder builder) {
        for (final var entry : motors.entrySet()) {
            final var motor = entry.getKey();
            final var name = entry.getValue();
            builder.addDoubleProperty(
                    "Voltages/" + name + " (" + motor.getDeviceId() + ")",
                    motor::getBusVoltage,
                    v -> motor.setVoltage(Volts.of(v)));
            builder.addDoubleProperty(
                    "Stator Currents/" + name + " (" + motor.getDeviceId() + ")", motor::getOutputCurrent, null);
            builder.addDoubleProperty(
                    "Outputs/" + name + " (" + motor.getDeviceId() + ")", motor::getAppliedOutput, motor::set);
        }
    }

    public void addMotor(final SparkMax motor, final String name) {
        motors.put(motor, name);
    }
}
