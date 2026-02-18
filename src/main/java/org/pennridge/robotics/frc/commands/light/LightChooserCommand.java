package org.pennridge.robotics.frc.commands.light;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import org.pennridge.robotics.frc.subsystems.LightsSubsystem.LightRule;
import org.pennridge.robotics.frc.subsystems.LightsSubsystem.LightSegment;

@NullMarked
public class LightChooserCommand extends Command {
    private final CANdle candle;
    private final List<LightRule> lightRules = new ArrayList<>();
    private final @Nullable LightRequest[] currentSlotRequests = new LightRequest[8];

    public LightChooserCommand(CANdle candle) {
        this.candle = candle;
    }

    /** The main body of a command. Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        checkCurrentSlotRequests();
        requestNewLightRules();
    }

    private void checkCurrentSlotRequests() {
        for (int i = 0; i < currentSlotRequests.length; i++) {
            final var request = currentSlotRequests[i];
            if (request == null) continue;
            if (request.originalRule.condition().getAsBoolean()) continue;
            currentSlotRequests[i] = null;
        }
    }

    private void requestNewLightRules() {
        lightRuleLoop:
        for (final var lightRule : lightRules) {
            var checkedCondition = false;
            for (final var entry : lightRule.requests().entrySet()) {
                final var segment = entry.getKey();
                final var requestSupplier = entry.getValue();
                if (currentSlotRequests[segment.slot()] != null) continue;
                if (!checkedCondition) {
                    if (!lightRule.condition().getAsBoolean()) continue lightRuleLoop;
                    checkedCondition = true;
                }
                final var request = requestSupplier.get();
                currentSlotRequests[segment.slot()] = new LightRequest(lightRule, segment);
                sendRequest(request);
            }
        }
    }

    public void addLightRule(LightRule rule) {
        lightRules.add(rule);
    }

    private void sendRequest(ControlRequest request) {
        final var statusCode = candle.setControl(request);
        if (statusCode == StatusCode.OK) return;
        DriverStation.reportError(
                "Could not run " + request.getName() + ": Status Code " + statusCode.getName() + " ("
                        + statusCode.getDescription() + ")",
                false);
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end() method and
     * un-schedule it.
     *
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Whether the given command should run when the robot is disabled. Override to return true if the command should
     * run when disabled.
     *
     * @return whether the command should run when the robot is disabled
     */
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    private record LightRequest(LightRule originalRule, LightSegment segment) {}
}
