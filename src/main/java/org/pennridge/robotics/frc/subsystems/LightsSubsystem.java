package org.pennridge.robotics.frc.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import org.pennridge.robotics.frc.commands.light.LightChooserCommand;
import org.pennridge.robotics.frc.util.enums.Constants.LightConstants;

@NullMarked
public class LightsSubsystem extends SubsystemBase {
    private final CANdle candle = new CANdle(LightConstants.CANDLE_ID);
    private final LightChooserCommand chooserCommand = new LightChooserCommand(candle);

    private final SwerveSubsystem swerveSubsystem;
    private final FuelSubsystem fuelSubsystem;

    public LightsSubsystem(SwerveSubsystem swerveSubsystem, FuelSubsystem fuelSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.fuelSubsystem = fuelSubsystem;
        final var config = new CANdleConfiguration()
                .withCANdleFeatures(new CANdleFeaturesConfigs()
                        .withEnable5VRail(Enable5VRailValue.Enabled)
                        .withVBatOutputMode(VBatOutputModeValue.Off)
                        .withStatusLedWhenActive(StatusLedWhenActiveValue.Enabled))
                .withLED(new LEDConfigs()
                        .withStripType(StripTypeValue.GRB)
                        .withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning)
                        .withBrightnessScalar(1.0));
        candle.getConfigurator().apply(config);

        CommandScheduler.getInstance().schedule(chooserCommand);

        addRules();
    }

    private void addRules() {
        // E-stop
        addStrobeAnimationRule(
                LightSegment.ALL,
                new RGBWColor(Color.kRed),
                (animation) -> {
                    animation.withFrameRate(2); // this is optional; see code below, for example
                },
                DriverStation::isEStopped);

        // This code does the same thing as above, except that it doesn't configure the framerate (default is 4)
        // addStrobeAnimationRule(LightSegment.ALL, new RGBWColor(Color.kRed), null, DriverStation::isEStopped);

        // Add others here (note that order matters!)

        // When disabled
        // addStrobeAnimationRule(LightSegment.ALL, new RGBWColor(255, 255, 255), null, SwerveSubsystem.);

        // X seconds left
        // addStrobeAnimationRule(LightSegment.ALL, new RGBWColor(255, 255, 255), () -> /*Logic here*/);

        // Bump Lock (overidden)
        addStrobeAnimationRule(LightSegment.ALL, new RGBWColor(0, 255, 255), null, () -> SwerveSubsystem.isBumpLockOverriddenTrigger());

        // Bump Lock
        addSolidColorRule(LightSegment.ALL, new RGBWColor(0, 255, 255), () -> SwerveSubsystem.isOnBumpTrigger());

        // When spinning up


        // When shooting
        // addStrobeAnimationRule(LightSegment.ALL, /*Wind-Up Color*/, /*If shooting*/);

        // When ejecting
        addStrobeAnimationRule(LightSegment.ALL, new RGBWColor(255, 255, 0), null, () -> FuelSubsystem.isEjectingTrigger());

        // When intaking
        addSolidColorRule(LightSegment.ALL, new RGBWColor(new Color("8702fc")), () -> FuelSubsystem.isIntakingTrigger());

        // When climbing
        // addSolidColorRule(LightSegment.ALL, new RGBWColor(new Color("f47718")), () -> /*If climbing*/);

        // When doing nothing
        addSolidColorRule(LightSegment.ALL, new RGBWColor(new Color("025b35")), () -> true);

    }

    private void addSolidColorRule(List<LightSegment> segments, RGBWColor color, BooleanSupplier condition) {
        final var requests = new HashMap<LightSegment, SolidColor>(segments.size());
        for (final var segment : segments) {
            requests.put(segment, new SolidColor(segment.startIndex, segment.endIndex).withColor(color));
        }
        addLightRule(requests, null, condition);
    }

    private void addClearAnimationRule(List<LightSegment> segments, BooleanSupplier condition) {
        final var requests = new HashMap<LightSegment, EmptyAnimation>(segments.size());
        for (final var segment : segments) {
            requests.put(segment, new EmptyAnimation(segment.slot));
        }
        addLightRule(requests, null, condition);
    }

    private void addColorFlowAnimationRule(
            List<LightSegment> segments,
            RGBWColor color,
            @Nullable Consumer<ColorFlowAnimation> configure,
            BooleanSupplier condition) {
        final var requests = new HashMap<LightSegment, ColorFlowAnimation>(segments.size());
        for (final var segment : segments) {
            requests.put(
                    segment,
                    new ColorFlowAnimation(segment.startIndex, segment.endIndex)
                            .withColor(color)
                            .withFrameRate(30) // 1 LED movement per frame
                            .withSlot(segment.slot));
        }
        addLightRule(requests, configure, condition);
    }

    private void addFireAnimationRule(
            List<LightSegment> segments, @Nullable Consumer<FireAnimation> configure, BooleanSupplier condition) {
        final var requests = new HashMap<LightSegment, FireAnimation>(segments.size());
        for (final var segment : segments) {
            requests.put(
                    segment,
                    new FireAnimation(segment.startIndex, segment.endIndex)
                            .withBrightness(1.0)
                            .withSparking(0.6)
                            .withCooling(0.3)
                            .withFrameRate(60)
                            .withSlot(segment.slot));
        }
        addLightRule(requests, configure, condition);
    }

    /** Animation that bounces a pocket of light across the LED strip. */
    private void addLarsonAnimationRule(
            List<LightSegment> segments, @Nullable Consumer<LarsonAnimation> configure, BooleanSupplier condition) {
        final var requests = new HashMap<LightSegment, LarsonAnimation>(segments.size());
        for (final var segment : segments) {
            requests.put(
                    segment,
                    new LarsonAnimation(segment.startIndex, segment.endIndex)
                            .withSize(3)
                            .withBounceMode(LarsonBounceValue.Front)
                            .withFrameRate(30) // 1 LED movement per frame
                            .withSlot(segment.slot));
        }
        addLightRule(requests, configure, condition);
    }

    private void addRainbowAnimationRule(
            List<LightSegment> segments, @Nullable Consumer<RainbowAnimation> configure, BooleanSupplier condition) {
        final var requests = new HashMap<LightSegment, RainbowAnimation>(segments.size());
        for (final var segment : segments) {
            requests.put(
                    segment,
                    new RainbowAnimation(segment.startIndex, segment.endIndex)
                            .withBrightness(1.0)
                            .withFrameRate(100) // ~3° per frame
                            .withSlot(segment.slot));
        }
        addLightRule(requests, configure, condition);
    }

    private void addRgbFadeAnimationRule(
            List<LightSegment> segments, @Nullable Consumer<RgbFadeAnimation> configure, BooleanSupplier condition) {
        final var requests = new HashMap<LightSegment, RgbFadeAnimation>(segments.size());
        for (final var segment : segments) {
            requests.put(
                    segment,
                    new RgbFadeAnimation(segment.startIndex, segment.endIndex)
                            .withBrightness(1.0)
                            .withFrameRate(100) // 1% brightness per frame
                            .withSlot(segment.slot));
        }
        addLightRule(requests, configure, condition);
    }

    private void addSingleFadeAnimationRule(
            List<LightSegment> segments,
            RGBWColor color,
            @Nullable Consumer<SingleFadeAnimation> configure,
            BooleanSupplier condition) {
        final var requests = new HashMap<LightSegment, SingleFadeAnimation>(segments.size());
        for (final var segment : segments) {
            requests.put(
                    segment,
                    new SingleFadeAnimation(segment.startIndex, segment.endIndex)
                            .withColor(color)
                            .withFrameRate(100) // 1% brightness per frame
                            .withSlot(segment.slot));
        }
        addLightRule(requests, configure, condition);
    }

    private void addStrobeAnimationRule(
            List<LightSegment> segments,
            RGBWColor color,
            @Nullable Consumer<StrobeAnimation> configure,
            BooleanSupplier condition) {
        final var requests = new HashMap<LightSegment, StrobeAnimation>(segments.size());
        for (final var segment : segments) {
            requests.put(
                    segment,
                    new StrobeAnimation(segment.startIndex, segment.endIndex)
                            .withColor(color)
                            .withFrameRate(4) // All LEDs on/off per frame
                            .withSlot(segment.slot));
        }
        addLightRule(requests, configure, condition);
    }

    /** Animation that randomly turns LEDs on and off to a certain color. */
    private void runTwinkleAnimationRule(
            List<LightSegment> segments,
            RGBWColor color,
            @Nullable Consumer<TwinkleAnimation> configure,
            BooleanSupplier condition) {
        final var requests = new HashMap<LightSegment, TwinkleAnimation>(segments.size());
        for (final var segment : segments) {
            requests.put(
                    segment,
                    new TwinkleAnimation(segment.startIndex, segment.endIndex)
                            .withColor(color)
                            .withMaxLEDsOnProportion(0.5)
                            .withFrameRate(100) // 1 LED on/off per frame
                            .withSlot(segment.slot));
        }
        addLightRule(requests, configure, condition);
    }

    /** Animation that randomly turns on LEDs until it reaches the maximum count and then turns them all off. */
    private void addTwinkleOffAnimationRule(
            List<LightSegment> segments,
            RGBWColor color,
            @Nullable Consumer<TwinkleOffAnimation> configure,
            BooleanSupplier condition) {
        final var requests = new HashMap<LightSegment, TwinkleOffAnimation>(segments.size());
        for (final var segment : segments) {
            requests.put(
                    segment,
                    new TwinkleOffAnimation(segment.startIndex, segment.endIndex)
                            .withColor(color)
                            .withMaxLEDsOnProportion(1.0)
                            .withFrameRate(25) // 1 LED on (or all off) per frame
                            .withSlot(segment.slot));
        }
        addLightRule(requests, configure, condition);
    }

    private <T extends ControlRequest> void addLightRule(
            Map<LightSegment, T> animations, @Nullable Consumer<T> config, BooleanSupplier condition) {
        final var requests = new HashMap<LightSegment, Supplier<? extends ControlRequest>>();
        for (final var entry : animations.entrySet()) {
            requests.put(entry.getKey(), () -> {
                if (config != null) {
                    config.accept(entry.getValue());
                }
                return entry.getValue();
            });
        }
        addLightRule(new LightRule(requests, condition));
    }

    private void addLightRule(LightRule lightRule) {
        chooserCommand.addLightRule(lightRule);
    }

    public record LightSegment(int startIndex, int endIndex, int slot) {
        // index can be from 8-399 (0-7 are for the built-in LEDs)
        // slot must be between 0-7

        public static final List<LightSegment> ALL = List.of(new LightSegment(8, 8, 0));
    }

    public record LightRule(
            Map<LightSegment, Supplier<? extends ControlRequest>> requests, BooleanSupplier condition) {}
}
