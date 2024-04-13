package frc.robot.subsystems.intake;

import frc.robot.Robot;
import frc.robot.lib.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends AdvancedSubsystem {

    private static IntakeSubsystem INSTANCE = new IntakeSubsystem();

    private final IntakeIO intake;

    private IntakeStates state;

    private IntakeSubsystem() {

        if (Robot.isReal()) {
            intake = new IntakeReal();
        }else{
            intake = new IntakeSim();
        }

        state = IntakeStates.off;

    }

    @Override
    public void readPeriodic() {
        intake.readPeriodic();
    }

    @Override
    public void writePeriodic() {

        intake.writePeriodic();
        Logger.recordOutput("Intake State", state.toString());
    }


    public void setState(IntakeStates state) {
        this.state = state;
        switch (state)
        {
            case fwd -> {
                intake.setVoltage(IntakeConstants.INTAKE_MAX_SPEED * 12);
            }
            case rev -> {
                intake.setVoltage(-IntakeConstants.INTAKE_MAX_SPEED * 12);
            }
            case fire -> {
                intake.setVoltage(IntakeConstants.INTAKE_SHOOT_SPEED * 12);
            }
            case off -> {
                intake.setVoltage(0);
            }
        }
    }

    public static IntakeSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakeSubsystem();
        }
        return INSTANCE;
    }

}

