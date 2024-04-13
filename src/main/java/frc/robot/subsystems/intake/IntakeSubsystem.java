package frc.robot.subsystems.intake;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private static IntakeSubsystem INSTANCE = new IntakeSubsystem();

    private final IntakeIO intake;

    private IntakeStates state;

    private double manualSpeed;

    private IntakeSubsystem() {

        if (Robot.isReal()) {
            intake = new IntakeReal();
        }else{
            intake = new IntakeSim();
        }

        state = IntakeStates.off;
        manualSpeed = 0;

    }

    public void periodic() {


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
            case manual -> {
                intake.setVoltage(getManualSpeed() * 12);
            }
            case off -> {
                intake.setVoltage(0);
            }
        }
        intake.update();
        Logger.recordOutput("Intake State", state.toString());

    }

    public void setState(IntakeStates state) {
        this.state = state;
    }

    public static IntakeSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakeSubsystem();
        }
        return INSTANCE;
    }

    public double getManualSpeed() {
        return manualSpeed;
    }

    public void setManualSpeed(double manualSpeed) {
        this.manualSpeed = manualSpeed;
    }
}

