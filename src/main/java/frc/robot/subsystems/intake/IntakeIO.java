package frc.robot.subsystems.intake;

public interface IntakeIO {

    void setVoltage(double voltage);

    void writePeriodic();
    void readPeriodic();
}
