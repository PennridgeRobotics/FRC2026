package frc.robot.util;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.core.CorePigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.enums.Constants;
import frc.robot.util.enums.Constants.PhysicalConstants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;

public class BumpManager {
    private boolean rawBumpLockEnabled; // doesn't take into account forceNormalDriveMode
    private boolean manualBumpLock;
    private boolean autoBumpLockPermitted = true;

    private final Trigger inBumpZoneTrigger;
    private final Trigger onBumpTrigger;
    private final Trigger bumpLockOverriddenTrigger;
    private final Trigger rawBumpLockEnabledTrigger; // doesn't take into account forceNormalDriveMode
    private final Trigger bumpLockEnabledTrigger; // takes into account forceNormalDriveMode
    private final Trigger manualBumpLockTrigger;
    private final Trigger autoBumpLockPermittedTrigger;

    private final Supplier<Pose2d> poseSupplier;

    private final BooleanSubscriber autoBumpLockPermittedSubscriber;

    public BumpManager(
            CorePigeon2 pigeon2,
            Supplier<Rotation3d> rotation3dSupplier,
            Supplier<Pose2d> poseSupplier,
            BooleanSupplier forceNormalDriveMode) {
        this.poseSupplier = poseSupplier;

        manualBumpLockTrigger = new Trigger(() -> manualBumpLock);
        autoBumpLockPermittedTrigger = new Trigger(() -> autoBumpLockPermitted);

        rawBumpLockEnabledTrigger = new Trigger(() -> rawBumpLockEnabled);
        bumpLockEnabledTrigger = rawBumpLockEnabledTrigger
                .and(autoBumpLockPermittedTrigger)
                .or(() -> manualBumpLock)
                .and(() -> !forceNormalDriveMode.getAsBoolean());

        inBumpZoneTrigger = new Trigger(this::isInBumpZone).debounce(0.1);
        inBumpZoneTrigger.onTrue(updateBumpLock(true, () -> "entered bump zone"));
        inBumpZoneTrigger.onFalse(updateBumpLock(false, () -> "left bump zone"));

        onBumpTrigger = inBumpZoneTrigger
                .and(() -> {
                    final var rotation3d = rotation3dSupplier.get();
                    final var angle =
                            Math.toDegrees(new Rotation3d(rotation3d.getX(), rotation3d.getY(), 0).getAngle());
                    final var rollVel = Math.abs(
                            pigeon2.getAngularVelocityXWorld(false).getValue().in(DegreesPerSecond));
                    final var pitchVel = Math.abs(
                            pigeon2.getAngularVelocityYWorld(false).getValue().in(DegreesPerSecond));
                    final var angularVelocity = Math.hypot(rollVel, pitchVel);
                    return angle > 7.0 || (angle > 2.0 && angularVelocity > 50);
                })
                .debounce(0.25, Debouncer.DebounceType.kBoth);
        onBumpTrigger.onTrue(Commands.runOnce(() -> System.out.println("on bump")));
        onBumpTrigger.onFalse(
                updateBumpLock(false, () -> inBumpZoneTrigger.getAsBoolean() ? "no longer on bump" : null));

        bumpLockOverriddenTrigger = rawBumpLockEnabledTrigger.and(forceNormalDriveMode);

        final var topicPrefix = "Bump/";
        final BooleanPublisher inBumpZonePublisher = NetworkTableInstance.getDefault()
                .getBooleanTopic(topicPrefix + "In Bump Zone")
                .publish();
        final BooleanPublisher onBumpPublisher = NetworkTableInstance.getDefault()
                .getBooleanTopic(topicPrefix + "On Bump")
                .publish();
        final BooleanPublisher bumpLockOverriddenPublisher = NetworkTableInstance.getDefault()
                .getBooleanTopic(topicPrefix + "Bump Lock Overridden")
                .publish();
        final BooleanPublisher rawBumpLockEnabledPublisher = NetworkTableInstance.getDefault()
                .getBooleanTopic(topicPrefix + "Raw Bump Lock Enabled")
                .publish();
        final BooleanPublisher bumpLockEnabledPublisher = NetworkTableInstance.getDefault()
                .getBooleanTopic(topicPrefix + "Bump Lock Enabled")
                .publish();
        final BooleanPublisher manualBumpLockPublisher = NetworkTableInstance.getDefault()
                .getBooleanTopic(topicPrefix + "Manual Bump Lock")
                .publish();
        final BooleanPublisher autoBumpLockPermittedPublisher = NetworkTableInstance.getDefault()
                .getBooleanTopic(topicPrefix + "Automatic Bump Lock Permitted")
                .publish();
        linkTriggerToPublisher(inBumpZoneTrigger, inBumpZonePublisher);
        linkTriggerToPublisher(onBumpTrigger, onBumpPublisher);
        linkTriggerToPublisher(bumpLockOverriddenTrigger, bumpLockOverriddenPublisher);
        linkTriggerToPublisher(rawBumpLockEnabledTrigger, rawBumpLockEnabledPublisher);
        linkTriggerToPublisher(bumpLockEnabledTrigger, bumpLockEnabledPublisher);
        linkTriggerToPublisher(manualBumpLockTrigger, manualBumpLockPublisher);
        linkTriggerToPublisher(autoBumpLockPermittedTrigger, autoBumpLockPermittedPublisher);
        autoBumpLockPermittedSubscriber =
                autoBumpLockPermittedPublisher.getTopic().subscribe(autoBumpLockPermitted);
    }

    public void periodic() {
        autoBumpLockPermitted = autoBumpLockPermittedSubscriber.get();
    }

    private void linkTriggerToPublisher(Trigger trigger, BooleanPublisher publisher) {
        trigger.onTrue(Commands.runOnce(() -> publisher.set(true)));
        trigger.onFalse(Commands.runOnce(() -> publisher.set(false)));
    }

    private Command updateBumpLock(boolean bumpLockEnabled, @Nullable Supplier<@Nullable String> cause) {
        return Commands.runOnce(() -> {
            if (cause != null && cause.get() != null) {
                System.out.println("Updated bump lock to " + bumpLockEnabled + " (" + cause.get() + ")");
            }
            this.rawBumpLockEnabled = bumpLockEnabled;
        });
    }

    public Rotation2d getBumpLockAngle() {
        final var botLength = PhysicalConstants.WHEEL_CENTERS_DISTANCE_LENGTH_X.in(Meters);
        final var botWidth = PhysicalConstants.WHEEL_CENTERS_DISTANCE_WIDTH_Y.in(Meters);

        final var angleDegrees = Math.toDegrees(Math.atan2(botLength, botWidth));

        // candidates: +theta, -theta, 180 - theta, theta - 180 (normalized equivalents of the four diagonal directions)
        final double[] candidates = new double[] {
            angleDegrees, // +theta
            -angleDegrees, // -theta
            180.0 - angleDegrees, // 180 - theta
            angleDegrees - 180.0 // theta - 180
        };

        final double currentDeg = poseSupplier.get().getRotation().getDegrees();
        double best = candidates[0];
        double bestAbsDiff = Math.abs(MathUtil.inputModulus(currentDeg - best, -180.0, 180.0));
        for (int i = 1; i < candidates.length; ++i) {
            final double cand = candidates[i];
            final double diff = Math.abs(MathUtil.inputModulus(currentDeg - cand, -180.0, 180.0));
            if (diff < bestAbsDiff) {
                bestAbsDiff = diff;
                best = cand;
            }
        }

        return Rotation2d.fromDegrees(best);
    }

    private boolean isInBumpZone() {
        final var pose = poseSupplier.get().getTranslation();
        for (var zone : Constants.FieldConstants.BUMP_ZONES) {
            if (zone.contains(pose)) {
                return true;
            }
        }
        return false;
    }

    public Trigger isInBumpZoneTrigger() {
        return inBumpZoneTrigger;
    }

    public Trigger isOnBumpTrigger() {
        return onBumpTrigger;
    }

    public Trigger isBumpLockOverriddenTrigger() {
        return bumpLockOverriddenTrigger;
    }

    public Trigger isRawBumpLockEnabledTrigger() {
        return rawBumpLockEnabledTrigger;
    }

    public Trigger isBumpLockEnabledTrigger() {
        return bumpLockEnabledTrigger;
    }

    public Command enableManualBumpLock() {
        return Commands.startEnd(() -> manualBumpLock = true, () -> manualBumpLock = false);
    }
}
