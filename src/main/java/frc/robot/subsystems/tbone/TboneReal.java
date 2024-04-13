package frc.robot.subsystems.tbone;

import com.revrobotics.*;
import org.littletonrobotics.junction.Logger;

public class TboneReal implements TBoneIO {

    private CANSparkMax motor;

    private double inputVoltage;

    private RelativeEncoder encoder;

    public TboneReal() {

        motor = new CANSparkMax(TboneConstants.TBONE_MOTOR_ID,
                CANSparkLowLevel.MotorType.kBrushed);

        motor.setInverted(true);

        motor.enableVoltageCompensation(12);

        motor.clearFaults();

        motor.setSmartCurrentLimit(40);
        encoder = motor.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 8192);


        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        inputVoltage = 0;
        setVoltage(0);
    }

    @Override
    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    public void setVoltage(double voltage) {
        inputVoltage = voltage;
        motor.set(inputVoltage);
    }

    @Override
    public void readPeriodic() {

    }

    @Override
    public void writePeriodic() {
        Logger.recordOutput("T-Bone/Position",encoder.getPosition());
        Logger.recordOutput("T-Bone/Voltage", inputVoltage);

    }
}
