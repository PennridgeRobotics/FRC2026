package frc.robot.util.controller;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.jspecify.annotations.NullMarked;

@NullMarked
public class CommandJoystickController extends CommandGenericHID {
    private final JoystickController m_hid;

    public CommandJoystickController(int port) {
        super(port);
        m_hid = new JoystickController(port);
    }

    @Override
    public JoystickController getHID() {
        return m_hid;
    }

    public Trigger flapsUp() {
        return flapsUp(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger flapsUp(EventLoop loop) {
        return button(JoystickController.Button.FlapsUp.getValue(), loop);
    }

    public Trigger flapsDown() {
        return flapsDown(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger flapsDown(EventLoop loop) {
        return button(JoystickController.Button.FlapsDown.getValue(), loop);
    }

    public Trigger a1() {
        return a1(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger a1(EventLoop loop) {
        return button(JoystickController.Button.A1.getValue(), loop);
    }

    public Trigger a2() {
        return a2(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger a2(EventLoop loop) {
        return button(JoystickController.Button.A2.getValue(), loop);
    }

    public Trigger a3() {
        return a3(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger a3(EventLoop loop) {
        return button(JoystickController.Button.A3.getValue(), loop);
    }

    public Trigger b1() {
        return b1(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger b1(EventLoop loop) {
        return button(JoystickController.Button.B1.getValue(), loop);
    }

    public Trigger b2() {
        return b2(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger b2(EventLoop loop) {
        return button(JoystickController.Button.B2.getValue(), loop);
    }

    public Trigger b3() {
        return b3(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger b3(EventLoop loop) {
        return button(JoystickController.Button.B3.getValue(), loop);
    }

    public Trigger c1() {
        return c1(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger c1(EventLoop loop) {
        return button(JoystickController.Button.C1.getValue(), loop);
    }

    public Trigger c2() {
        return c2(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger c2(EventLoop loop) {
        return button(JoystickController.Button.C2.getValue(), loop);
    }

    public Trigger c3() {
        return c3(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger c3(EventLoop loop) {
        return button(JoystickController.Button.C3.getValue(), loop);
    }

    public Trigger start() {
        return start(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger start(EventLoop loop) {
        return button(JoystickController.Button.Start.getValue(), loop);
    }

    public Trigger eject() {
        return eject(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger eject(EventLoop loop) {
        return button(JoystickController.Button.Eject.getValue(), loop);
    }

    public Trigger trigger(boolean alt, boolean shift) {
        return trigger(alt, shift, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger trigger(boolean alt, boolean shift, EventLoop loop) {
        if (alt && shift) {
            return button(JoystickController.Button.TriggerAltShift.getValue(), loop);
        } else if (alt) {
            return button(JoystickController.Button.TriggerAlt.getValue(), loop);
        } else if (shift) {
            return button(JoystickController.Button.TriggerShift.getValue(), loop);
        } else {
            return button(JoystickController.Button.Trigger.getValue(), loop);
        }
    }

    public Trigger topHat(boolean alt, boolean shift) {
        return topHat(alt, shift, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger topHat(boolean alt, boolean shift, EventLoop loop) {
        if (alt && shift) {
            return button(JoystickController.Button.TopHatAltShift.getValue(), loop);
        } else if (alt) {
            return button(JoystickController.Button.TopHatAlt.getValue(), loop);
        } else if (shift) {
            return button(JoystickController.Button.TopHatShift.getValue(), loop);
        } else {
            return button(JoystickController.Button.TopHat.getValue(), loop);
        }
    }

    /** Right is positive */
    public double getX() {
        return m_hid.getX();
    }

    /** Back is positive */
    public double getY() {
        return m_hid.getY();
    }

    public double getThrottle() {
        return m_hid.getThrottle();
    }

    /** Clockwise is positive * */
    public double getTwist() {
        return m_hid.getTwist();
    }
}
