package frc.robot.IO;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public abstract class IO {
    public enum PrimaryDriverProfiles {
        Leo,
    }
    public enum SecondaryDriverProfiles {
        KnollJoystick,
        KnollController
    }

    private static final XboxController primary = new XboxController(0);
    private static final XboxController secondary = new XboxController(1);
    private static final Joystick secondaryJoystick  = new Joystick(1);

    private static final HashMap<Controls, Supplier<Double>> controlsJoystick = new HashMap<>();
    private static final HashMap<Controls, BooleanSupplier> controlsButtons = new HashMap<>();

    public static void Initialize(PrimaryDriverProfiles primaryProfile, SecondaryDriverProfiles secondaryProfile) {
        controlsJoystick.clear();
        controlsButtons.clear();

        switch(primaryProfile) {
            case Leo:
                controlsJoystick.put(Controls.drive_x, primary::getLeftX);
                controlsJoystick.put(Controls.drive_y, primary::getLeftY);
                controlsJoystick.put(Controls.drive_theta, primary::getRightX);

                controlsJoystick.put(Controls.headingX, primary::getLeftX);
                controlsJoystick.put(Controls.headingY, primary::getLeftY);
                controlsJoystick.put(Controls.throttle, primary::getRightTriggerAxis);

                controlsButtons.put(Controls.reset_gyro, primary::getLeftBumper);
                controlsButtons.put(Controls.slowMode, primary::getRightBumper);
                controlsButtons.put(Controls.normalMode, () -> (.75 < primary.getRightTriggerAxis()));

                controlsJoystick.put(Controls.thetaX, primary::getRightX);
                controlsJoystick.put(Controls.thetaY, primary::getRightY);
                controlsButtons.put(Controls.turningModeToggle, primary::getRightStickButton);
                break;
        }

        switch (secondaryProfile) {
            case KnollJoystick:
                controlsButtons.put(Controls.intakeFWD, () -> secondaryJoystick.getRawButton(3));
                controlsButtons.put(Controls.intakeREVS, () -> secondaryJoystick.getRawAxis(2) > .5 || secondaryJoystick.getRawAxis(2) < -.5);

                controlsButtons.put(Controls.fire, () -> secondaryJoystick.getRawButton(1));
                controlsButtons.put(Controls.revSpeaker, () -> secondaryJoystick.getRawAxis(1) > .6);
                controlsButtons.put(Controls.revTape, () -> secondaryJoystick.getRawAxis(1) < -.6);

                controlsButtons.put(Controls.AmpScore, () -> secondaryJoystick.getX() > .5 || secondaryJoystick.getX() < -.5);

                controlsButtons.put(Controls.climb, () -> (secondary.getPOV() == 90));
                controlsButtons.put(Controls.invertClimb, () -> (secondary.getPOV() == 270));
                controlsButtons.put(Controls.releaseClimber, () -> (secondary.getPOV() == 0));
                controlsButtons.put(Controls.pinClimber, () -> (secondary.getPOV() == 180));

                controlsButtons.put(Controls.toggleDeflector, () -> secondaryJoystick.getRawButton(2));

                controlsButtons.put(Controls.sourceIntake, () -> secondaryJoystick.getRawButton(4));

                controlsButtons.put(Controls.toggleTBone, () -> secondaryJoystick.getRawButton(14));
                break;
            case KnollController:
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
                break;
        }
    }

    public static Supplier<Double> getJoystickValue(Controls control) {
        return controlsJoystick.get(control);
    }

    public static BooleanSupplier getButtonValue(Controls control) {
        return controlsButtons.get(control);
    }

    public static double getDPadPrimary()
    {
        return primary.getPOV();
    }
}

