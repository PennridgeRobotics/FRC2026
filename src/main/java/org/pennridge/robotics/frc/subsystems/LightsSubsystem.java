package org.pennridge.robotics.frc.subsystems;

import com.ctre.phoenix6.StatusCode;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import org.pennridge.robotics.frc.util.enums.Constants.LightConstants;

@NullMarked
public class LightsSubsystem extends SubsystemBase {
    private final CANdle candle = new CANdle(LightConstants.CANDLE_ID);

    public LightsSubsystem() {
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

        setDefaultCommand(setSolidColor(LightSegment.ALL, RGBWColor.fromHSV(120, 1, 0.5)));
    }

    public Command setSolidColor(List<LightSegment> segments, RGBWColor color) {
        final var requests = new ArrayList<SolidColor>(segments.size());
        for (final var segment : segments) {
            requests.add(new SolidColor(segment.startIndex, segment.endIndex).withColor(color));
        }
        return runAnimation(requests, null);
    }

    public Command clearAnimation(List<LightSegment> segments) {
        final var requests = new ArrayList<EmptyAnimation>(segments.size());
        for (final var segment : segments) {
            requests.add(new EmptyAnimation(segment.slot));
        }
        return runAnimation(requests, null);
    }

    public Command runColorFlowAnimation(
            List<LightSegment> segments, RGBWColor color, @Nullable Consumer<ColorFlowAnimation> configure) {
        final var requests = new ArrayList<ColorFlowAnimation>(segments.size());
        for (final var segment : segments) {
            requests.add(new ColorFlowAnimation(segment.startIndex, segment.endIndex)
                    .withColor(color)
                    .withFrameRate(30) // 1 LED movement per frame
                    .withSlot(segment.slot));
        }
        return runAnimation(requests, configure);
    }

    public Command runFireAnimation(List<LightSegment> segments, @Nullable Consumer<FireAnimation> configure) {
        final var requests = new ArrayList<FireAnimation>(segments.size());
        for (final var segment : segments) {
            requests.add(new FireAnimation(segment.startIndex, segment.endIndex)
                    .withBrightness(1.0)
                    .withSparking(0.6)
                    .withCooling(0.3)
                    .withFrameRate(60)
                    .withSlot(segment.slot));
        }
        return runAnimation(requests, configure);
    }

    /** Animation that bounces a pocket of light across the LED strip. */
    public Command runLarsonAnimation(List<LightSegment> segments, @Nullable Consumer<LarsonAnimation> configure) {
        final var requests = new ArrayList<LarsonAnimation>(segments.size());
        for (final var segment : segments) {
            requests.add(new LarsonAnimation(segment.startIndex, segment.endIndex)
                    .withSize(3)
                    .withBounceMode(LarsonBounceValue.Front)
                    .withFrameRate(30) // 1 LED movement per frame
                    .withSlot(segment.slot));
        }
        return runAnimation(requests, configure);
    }

    public Command runRainbowAnimation(List<LightSegment> segments, @Nullable Consumer<RainbowAnimation> configure) {
        final var requests = new ArrayList<RainbowAnimation>(segments.size());
        for (final var segment : segments) {
            requests.add(new RainbowAnimation(segment.startIndex, segment.endIndex)
                    .withBrightness(1.0)
                    .withFrameRate(100) // ~3° per frame
                    .withSlot(segment.slot));
        }
        return runAnimation(requests, configure);
    }

    public Command runRgbFadeAnimation(List<LightSegment> segments, @Nullable Consumer<RgbFadeAnimation> configure) {
        final var requests = new ArrayList<RgbFadeAnimation>(segments.size());
        for (final var segment : segments) {
            requests.add(new RgbFadeAnimation(segment.startIndex, segment.endIndex)
                    .withBrightness(1.0)
                    .withFrameRate(100) // 1% brightness per frame
                    .withSlot(segment.slot));
        }
        return runAnimation(requests, configure);
    }

    public Command runSingleFadeAnimation(
            List<LightSegment> segments, RGBWColor color, @Nullable Consumer<SingleFadeAnimation> configure) {
        final var requests = new ArrayList<SingleFadeAnimation>(segments.size());
        for (final var segment : segments) {
            requests.add(new SingleFadeAnimation(segment.startIndex, segment.endIndex)
                    .withColor(color)
                    .withFrameRate(100) // 1% brightness per frame
                    .withSlot(segment.slot));
        }
        return runAnimation(requests, configure);
    }

    public Command runStrobeAnimation(
            List<LightSegment> segments, RGBWColor color, @Nullable Consumer<StrobeAnimation> configure) {
        final var requests = new ArrayList<StrobeAnimation>(segments.size());
        for (final var segment : segments) {
            requests.add(new StrobeAnimation(segment.startIndex, segment.endIndex)
                    .withColor(color)
                    .withFrameRate(4) // All LEDs on/off per frame
                    .withSlot(segment.slot));
        }
        return runAnimation(requests, configure);
    }

    /** Animation that randomly turns LEDs on and off to a certain color. */
    public Command runTwinkleAnimation(
            List<LightSegment> segments, RGBWColor color, @Nullable Consumer<TwinkleAnimation> configure) {
        final var requests = new ArrayList<TwinkleAnimation>(segments.size());
        for (final var segment : segments) {
            requests.add(new TwinkleAnimation(segment.startIndex, segment.endIndex)
                    .withColor(color)
                    .withMaxLEDsOnProportion(0.5)
                    .withFrameRate(100) // 1 LED on/off per frame
                    .withSlot(segment.slot));
        }
        return runAnimation(requests, configure);
    }

    /** Animation that randomly turns on LEDs until it reaches the maximum count and then turns them all off. */
    public Command runTwinkleOffAnimation(
            List<LightSegment> segments, RGBWColor color, @Nullable Consumer<TwinkleOffAnimation> configure) {
        final var requests = new ArrayList<TwinkleOffAnimation>(segments.size());
        for (final var segment : segments) {
            requests.add(new TwinkleOffAnimation(segment.startIndex, segment.endIndex)
                    .withColor(color)
                    .withMaxLEDsOnProportion(1.0)
                    .withFrameRate(25) // 1 LED on (or all off) per frame
                    .withSlot(segment.slot));
        }
        return runAnimation(requests, configure);
    }

    private <T extends ControlRequest> Command runAnimation(List<T> animations, @Nullable Consumer<T> config) {
        return Commands.runOnce(() -> {
            for (final var request : animations) {
                if (config != null) {
                    config.accept(request);
                }
                final var statusCode = candle.setControl(request);
                if (statusCode == StatusCode.OK) continue;
                DriverStation.reportError(
                        "Could not run " + request.getName() + ": Status Code " + statusCode.getName() + " ("
                                + statusCode.getDescription() + ")",
                        false);
            }
        });
    }

    public record LightSegment(int startIndex, int endIndex, int slot) {
        // index can be from 8-399 (0-7 are for the built-in LEDs)
        // slot must be between 0-7

        public static final List<LightSegment> ALL = List.of(new LightSegment(8, 8, 0));
    }
}
