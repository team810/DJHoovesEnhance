package frc.robot.subsystems.climber;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.lib.AdvancedSubsystem;

public class ClimberSubsystem extends AdvancedSubsystem {

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

        climber = new ClimberReal();
        state = ClimberStates.off;
    }

    @Override
    public void readPeriodic() {
        climber.readPeriodic();
    }

    @Override
    public void writePeriodic() {
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
        climber.writePeriodic();
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

