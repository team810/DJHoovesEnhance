package frc.robot.subsystems.climber;

public interface ClimberIO {

    void setVoltage(double voltage);
    void readPeriodic();
    void writePeriodic();
    void release();

    void pin();



}
