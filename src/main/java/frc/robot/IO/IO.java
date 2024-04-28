package frc.robot.IO;

import edu.wpi.first.wpilibj.XboxController;

import java.util.HashMap;
import java.util.function.Supplier;

public abstract class IO {

    private static final XboxController primary = new XboxController(0);
    private static final XboxController secondary = new XboxController(1);

    private static final HashMap<Controls, Supplier<Double>> controlsJoystick = new HashMap<>();
    private static final HashMap<Controls, Supplier<Boolean>> controlsButtons = new HashMap<>();

    public static void Initialize() {
        controlsJoystick.put(Controls.drive_x, primary::getLeftX);
        controlsJoystick.put(Controls.drive_y, primary::getLeftY);
        controlsJoystick.put(Controls.drive_theta, primary::getRightX);

        controlsJoystick.put(Controls.headingX, primary::getLeftX);
        controlsJoystick.put(Controls.headingY, primary::getLeftY);
        controlsJoystick.put(Controls.throttle, primary::getRightTriggerAxis);

        controlsButtons.put(Controls.reset_gyro, primary::getLeftBumper);
        controlsButtons.put(Controls.slowMode, primary::getRightBumper);
        controlsButtons.put(Controls.normalMode, () -> (.75 < primary.getRightTriggerAxis()));

        controlsButtons.put(Controls.autoAlignAmp, primary::getBButton);
        controlsButtons.put(Controls.autoAlignPodium, primary::getAButton);
        controlsButtons.put(Controls.autoAlignSource, primary::getXButton);

        controlsButtons.put(Controls.intakeFWD, secondary::getAButton);
        controlsButtons.put(Controls.intakeREVS, secondary::getYButton);

        controlsButtons.put(Controls.fire, () -> secondary.getRightTriggerAxis() > .75);
        controlsButtons.put(Controls.revSpeaker, secondary::getLeftBumper);
        controlsButtons.put(Controls.revTape, () -> secondary.getLeftTriggerAxis() > .75);

        controlsButtons.put(Controls.AmpScore, secondary::getRightBumper);

        controlsButtons.put(Controls.climb, () -> (secondary.getPOV() == 90));
        controlsButtons.put(Controls.invertClimb, () -> (secondary.getPOV() == 270));
        controlsButtons.put(Controls.releaseClimber, () -> (secondary.getPOV() == 0));
        controlsButtons.put(Controls.pinClimber, () -> (secondary.getPOV() == 180));

        controlsButtons.put(Controls.toggleDeflector, secondary::getLeftStickButton);

        controlsButtons.put(Controls.sourceIntake, secondary::getXButton);

        controlsButtons.put(Controls.toggleTBone, secondary::getRightStickButton);

    }

    public static Supplier<Double> getJoystickValue(Controls control) {
        return controlsJoystick.get(control);
    }

    public static Supplier<Boolean> getButtonValue(Controls control) {
        return controlsButtons.get(control);
    }

    public static double getDPadPrimary()
    {
        return primary.getPOV();
    }
}

