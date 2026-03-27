package frc.robot.util.controller;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class JoystickController extends GenericHID implements Sendable {
    public enum Button {
        FlapsUp(1),
        FlapsDown(2),
        A1(3),
        A2(4),
        A3(5),
        B1(6),
        B2(7),
        B3(8),
        C1(9),
        C2(10),
        C3(11),
        Start(12),
        Eject(13),
        Trigger(14),
        TriggerShift(15),
        TriggerAlt(16),
        TriggerAltShift(17),
        TopHat(18),
        TopHatShift(19),
        TopHatAlt(20),
        TopHatAltShift(21);

        private final int value;

        Button(int value) {
            this.value = value;
        }

        @Override
        public String toString() {
            return this.name() + "Button";
        }

        public int getValue() {
            return value;
        }
    }

    public enum Axis {
        /** Right = positive */
        X(0),
        /** Back = positive */
        Y(1),
        Throttle(2),
        /** Clockwise = positive */
        Twist(3);

        private final int value;

        Axis(int value) {
            this.value = value;
        }

        @Override
        public String toString() {
            return this.name() + "Axis";
        }
    }

    public JoystickController(final int port) {
        super(port);
    }

    // Axis
    /** Right is positive */
    public double getX() {
        return getRawAxis(Axis.X);
    }

    /** Back is positive */
    public double getY() {
        return getRawAxis(Axis.Y);
    }

    public double getThrottle() {
        return getRawAxis(Axis.Throttle);
    }

    /** Clockwise is positive */
    public double getTwist() {
        return getRawAxis(Axis.Twist);
    }

    public double getRawAxis(Axis axis) {
        return getRawAxis(axis.value);
    }

    // Buttons
    public boolean getFlapsUpButton() {
        return getRawButton(Button.FlapsUp);
    }

    public boolean getFlapsUpButtonPressed() {
        return getRawButtonPressed(Button.FlapsUp);
    }

    public boolean getFlapsUpButtonReleased() {
        return getRawButtonReleased(Button.FlapsUp);
    }

    public BooleanEvent flapsUp(EventLoop loop) {
        return button(Button.FlapsUp, loop);
    }

    public boolean getFlapsDownButton() {
        return getRawButton(Button.FlapsDown);
    }

    public boolean getFlapsDownButtonPressed() {
        return getRawButtonPressed(Button.FlapsDown);
    }

    public boolean getFlapsDownButtonReleased() {
        return getRawButtonReleased(Button.FlapsDown);
    }

    public BooleanEvent flapsDown(EventLoop loop) {
        return button(Button.FlapsDown, loop);
    }

    public boolean getA1Button() {
        return getRawButton(Button.A1);
    }

    public boolean getA1ButtonPressed() {
        return getRawButtonPressed(Button.A1);
    }

    public boolean getA1ButtonReleased() {
        return getRawButtonReleased(Button.A1);
    }

    public BooleanEvent a1(EventLoop loop) {
        return button(Button.A1, loop);
    }

    public boolean getA2Button() {
        return getRawButton(Button.A2);
    }

    public boolean getA2ButtonPressed() {
        return getRawButtonPressed(Button.A2);
    }

    public boolean getA2ButtonReleased() {
        return getRawButtonReleased(Button.A2);
    }

    public BooleanEvent a2(EventLoop loop) {
        return button(Button.A2, loop);
    }

    public boolean getA3Button() {
        return getRawButton(Button.A3);
    }

    public boolean getA3ButtonPressed() {
        return getRawButtonPressed(Button.A3);
    }

    public boolean getA3ButtonReleased() {
        return getRawButtonReleased(Button.A3);
    }

    public BooleanEvent a3(EventLoop loop) {
        return button(Button.A3, loop);
    }

    public boolean getB1Button() {
        return getRawButton(Button.B1);
    }

    public boolean getB1ButtonPressed() {
        return getRawButtonPressed(Button.B1);
    }

    public boolean getB1ButtonReleased() {
        return getRawButtonReleased(Button.B1);
    }

    public BooleanEvent b1(EventLoop loop) {
        return button(Button.B1, loop);
    }

    public boolean getB2Button() {
        return getRawButton(Button.B2);
    }

    public boolean getB2ButtonPressed() {
        return getRawButtonPressed(Button.B2);
    }

    public boolean getB2ButtonReleased() {
        return getRawButtonReleased(Button.B2);
    }

    public BooleanEvent b2(EventLoop loop) {
        return button(Button.B2, loop);
    }

    public boolean getB3Button() {
        return getRawButton(Button.B3);
    }

    public boolean getB3ButtonPressed() {
        return getRawButtonPressed(Button.B3);
    }

    public boolean getB3ButtonReleased() {
        return getRawButtonReleased(Button.B3);
    }

    public BooleanEvent b3(EventLoop loop) {
        return button(Button.B3, loop);
    }

    public boolean getC1Button() {
        return getRawButton(Button.C1);
    }

    public boolean getC1ButtonPressed() {
        return getRawButtonPressed(Button.C1);
    }

    public boolean getC1ButtonReleased() {
        return getRawButtonReleased(Button.C1);
    }

    public BooleanEvent c1(EventLoop loop) {
        return button(Button.C1, loop);
    }

    public boolean getC2Button() {
        return getRawButton(Button.C2);
    }

    public boolean getC2ButtonPressed() {
        return getRawButtonPressed(Button.C2);
    }

    public boolean getC2ButtonReleased() {
        return getRawButtonReleased(Button.C2);
    }

    public BooleanEvent c2(EventLoop loop) {
        return button(Button.C2, loop);
    }

    public boolean getC3Button() {
        return getRawButton(Button.C3);
    }

    public boolean getC3ButtonPressed() {
        return getRawButtonPressed(Button.C3);
    }

    public boolean getC3ButtonReleased() {
        return getRawButtonReleased(Button.C3);
    }

    public BooleanEvent c3(EventLoop loop) {
        return button(Button.C3, loop);
    }

    public boolean getStartButton() {
        return getRawButton(Button.Start);
    }

    public boolean getStartButtonPressed() {
        return getRawButtonPressed(Button.Start);
    }

    public boolean getStartButtonReleased() {
        return getRawButtonReleased(Button.Start);
    }

    public BooleanEvent start(EventLoop loop) {
        return button(Button.Start, loop);
    }

    public boolean getEjectButton() {
        return getRawButton(Button.Eject);
    }

    public boolean getEjectButtonPressed() {
        return getRawButtonPressed(Button.Eject);
    }

    public boolean getEjectButtonReleased() {
        return getRawButtonReleased(Button.Eject);
    }

    public BooleanEvent eject(EventLoop loop) {
        return button(Button.Eject, loop);
    }

    public boolean getTriggerButton(boolean alt, boolean shift) {
        if (alt && shift) {
            return getRawButton(Button.TriggerAltShift);
        } else if (alt) {
            return getRawButton(Button.TriggerAlt);
        } else if (shift) {
            return getRawButton(Button.TriggerShift);
        } else {
            return getRawButton(Button.Trigger);
        }
    }

    public boolean getTriggerButtonPressed(boolean alt, boolean shift) {
        if (alt && shift) {
            return getRawButtonPressed(Button.TriggerAltShift);
        } else if (alt) {
            return getRawButtonPressed(Button.TriggerAlt);
        } else if (shift) {
            return getRawButtonPressed(Button.TriggerShift);
        } else {
            return getRawButtonPressed(Button.Trigger);
        }
    }

    public boolean getTriggerButtonReleased(boolean alt, boolean shift) {
        if (alt && shift) {
            return getRawButtonReleased(Button.TriggerAltShift);
        } else if (alt) {
            return getRawButtonReleased(Button.TriggerAlt);
        } else if (shift) {
            return getRawButtonReleased(Button.TriggerShift);
        } else {
            return getRawButtonReleased(Button.Trigger);
        }
    }

    public BooleanEvent trigger(boolean alt, boolean shift, EventLoop loop) {
        if (alt && shift) {
            return button(Button.TriggerAltShift, loop);
        } else if (alt) {
            return button(Button.TriggerAlt, loop);
        } else if (shift) {
            return button(Button.TriggerShift, loop);
        } else {
            return button(Button.Trigger, loop);
        }
    }

    public boolean getTopHatButton(boolean alt, boolean shift) {
        if (alt && shift) {
            return getRawButton(Button.TopHatAltShift);
        } else if (alt) {
            return getRawButton(Button.TopHatAlt);
        } else if (shift) {
            return getRawButton(Button.TopHatShift);
        } else {
            return getRawButton(Button.TopHat);
        }
    }

    public boolean getTopHatButtonPressed(boolean alt, boolean shift) {
        if (alt && shift) {
            return getRawButtonPressed(Button.TopHatAltShift);
        } else if (alt) {
            return getRawButtonPressed(Button.TopHatAlt);
        } else if (shift) {
            return getRawButtonPressed(Button.TopHatShift);
        } else {
            return getRawButtonPressed(Button.TopHat);
        }
    }

    public boolean getTopHatButtonReleased(boolean alt, boolean shift) {
        if (alt && shift) {
            return getRawButtonReleased(Button.TopHatAltShift);
        } else if (alt) {
            return getRawButtonReleased(Button.TopHatAlt);
        } else if (shift) {
            return getRawButtonReleased(Button.TopHatShift);
        } else {
            return getRawButtonReleased(Button.TopHat);
        }
    }

    public BooleanEvent topHat(boolean alt, boolean shift, EventLoop loop) {
        if (alt && shift) {
            return button(Button.TopHatAltShift, loop);
        } else if (alt) {
            return button(Button.TopHatAlt, loop);
        } else if (shift) {
            return button(Button.TopHatShift, loop);
        } else {
            return button(Button.TopHat, loop);
        }
    }

    public boolean getRawButton(Button button) {
        return getRawButton(button.value);
    }

    public boolean getRawButtonPressed(Button button) {
        return getRawButtonPressed(button.value);
    }

    public boolean getRawButtonReleased(Button button) {
        return getRawButtonReleased(button.value);
    }

    public BooleanEvent button(Button button, EventLoop loop) {
        return button(button.value, loop);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("HID");
        builder.publishConstString("ControllerType", "Joystick");
        builder.addDoubleProperty("X Axis", this::getX, null);
        builder.addDoubleProperty("Y Axis", this::getY, null);
        builder.addDoubleProperty("Throttle Axis", this::getThrottle, null);
        builder.addDoubleProperty("Twist Axis", this::getTwist, null);
        builder.addBooleanProperty("FlapsUp", this::getFlapsUpButton, null);
        builder.addBooleanProperty("FlapsDown", this::getFlapsDownButton, null);
        builder.addBooleanProperty("A1", this::getA1Button, null);
        builder.addBooleanProperty("A2", this::getA2Button, null);
        builder.addBooleanProperty("A3", this::getA3Button, null);
        builder.addBooleanProperty("B1", this::getB1Button, null);
        builder.addBooleanProperty("B2", this::getB2Button, null);
        builder.addBooleanProperty("B3", this::getB3Button, null);
        builder.addBooleanProperty("C1", this::getC1Button, null);
        builder.addBooleanProperty("C2", this::getC2Button, null);
        builder.addBooleanProperty("C3", this::getC3Button, null);
        builder.addBooleanProperty("Start", this::getStartButton, null);
        builder.addBooleanProperty("Eject", this::getEjectButton, null);
        builder.addBooleanProperty("Trigger", () -> getTriggerButton(false, false), null);
        builder.addBooleanProperty("Trigger Alt", () -> getTriggerButton(true, false), null);
        builder.addBooleanProperty("Trigger Shift", () -> getTriggerButton(false, true), null);
        builder.addBooleanProperty("Trigger Alt Shift", () -> getTriggerButton(true, true), null);
        builder.addBooleanProperty("TopHat", () -> getTopHatButton(false, false), null);
        builder.addBooleanProperty("TopHat Alt", () -> getTopHatButton(true, false), null);
        builder.addBooleanProperty("TopHat Shift", () -> getTopHatButton(false, true), null);
        builder.addBooleanProperty("TopHat Alt Shift", () -> getTopHatButton(true, true), null);
    }
}
