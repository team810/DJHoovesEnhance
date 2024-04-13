package frc.robot.subsystems.tbone;

public interface TBoneIO {
    double getEncoderPosition();

    void setVoltage(double voltage);

    void readPeriodic();
    void writePeriodic();
}
