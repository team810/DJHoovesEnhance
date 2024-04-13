package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import org.littletonrobotics.junction.Logger;

public class IntakeReal implements IntakeIO {

    private final CANSparkMax topMotor;
    private final CANSparkMax bottomMotor;

    private double inputVoltage;

    public IntakeReal() {

        topMotor = new CANSparkMax(IntakeConstants.TOP_ID,
                CANSparkLowLevel.MotorType.kBrushless);

        bottomMotor = new CANSparkMax(IntakeConstants.BOTTOM_ID,
                CANSparkLowLevel.MotorType.kBrushless);

        topMotor.setInverted(true);

        topMotor.enableVoltageCompensation(12);
        bottomMotor.enableVoltageCompensation(12);

        topMotor.clearFaults();
        bottomMotor.clearFaults();

        topMotor.setSmartCurrentLimit(40);
        bottomMotor.setSmartCurrentLimit(40);

        topMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        bottomMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        inputVoltage = 0;
        setVoltage(0);
    }

    public void setVoltage(double voltage) {
        inputVoltage = voltage;
        topMotor.set(inputVoltage);
        bottomMotor.set(inputVoltage);
    }

    public void update() {
        Logger.recordOutput("Intake/Top/Temperature", topMotor.getMotorTemperature());
        Logger.recordOutput("Intake/Top/CurrentDraw", topMotor.getOutputCurrent());
        Logger.recordOutput("Intake/Top/MotorVoltage", topMotor.getBusVoltage());
        Logger.recordOutput("Intake/Top/InputVoltage", this.inputVoltage);

        Logger.recordOutput("Intake/Bottom/Temperature", bottomMotor.getMotorTemperature());
        Logger.recordOutput("Intake/Bottom/CurrentDraw", bottomMotor.getOutputCurrent());
        Logger.recordOutput("Intake/Bottom/MotorVoltage", bottomMotor.getBusVoltage());
        Logger.recordOutput("Intake/Bottom/InputVoltage", this.inputVoltage);
    }
}
