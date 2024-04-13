package frc.robot.subsystems.climber;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ClimberSubsystem extends SubsystemBase {

    private static ClimberSubsystem INSTANCE = new ClimberSubsystem();

    private ClimberIO climber;

    private ClimberStates state;

    public static ClimberSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ClimberSubsystem();
        }
        return INSTANCE;
    }

    private ClimberSubsystem() {

        if (Robot.isReal()) {
            climber = new ClimberReal();
        } else {
            climber = new ClimberSim();
        }

        state = ClimberStates.off;
    }

    public void periodic() {

        switch (state)
        {
            case down -> {
                climber.setVoltage(ClimberConstants.CLIMBER_SPEED * 12);
            }
            case up -> {
                climber.setVoltage(-ClimberConstants.CLIMBER_SPEED * 12);
            }
            case off -> {
                climber.setVoltage(0);
            }
        }
        climber.update();
    }

    public void releaseClimber()
    {
        climber.release();
    }

    public void pinClimber() { climber.pin(); }

    public void setState(ClimberStates state) {
        this.state = state;
    }
}

