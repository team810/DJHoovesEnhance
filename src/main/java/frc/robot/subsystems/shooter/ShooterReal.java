package frc.robot.subsystems.shooter;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.RobotState;
import org.littletonrobotics.junction.Logger;

public class ShooterReal implements ShooterIO {
    private final CANSparkMax topMotor;
    private final CANSparkMax bottomMotor;

    private final RelativeEncoder topEncoder;
    private final RelativeEncoder bottomEncoder;

    private final SparkPIDController topController;
    private final SparkPIDController bottomController;

    private double topTargetRPM;
    private double bottomTargetRPM;


    public ShooterReal() {
        topMotor = new CANSparkMax(ShooterConstants.TOP_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        bottomMotor = new CANSparkMax(ShooterConstants.BOTTOM_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

        topMotor.setSmartCurrentLimit(40);
        bottomMotor.setSmartCurrentLimit(40);

        topMotor.enableVoltageCompensation(12);
        bottomMotor.enableVoltageCompensation(12);

        topMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        bottomMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);

        topMotor.clearFaults();
        bottomMotor.clearFaults();

        topEncoder = topMotor.getEncoder();
        bottomEncoder = bottomMotor.getEncoder();

        topMotor.setInverted(true);

        topController = topMotor.getPIDController();
        bottomController = bottomMotor.getPIDController();

        topController.setP(0.00006);
        topController.setI(0);
        topController.setD(0);
        topController.setFF(0.000185);

        bottomController.setP(0.00006);
        bottomController.setI(0);
        bottomController.setD(0);
        bottomController.setFF(0.000185);


    }
    @Override
    public void readPeriodic() {
        Logger.recordOutput("Shooter/Top/CurrentDraw", topMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/Top/Temperature", topMotor.getMotorTemperature());
        Logger.recordOutput("Shooter/Top/Velocity", topEncoder.getVelocity());
        Logger.recordOutput("Shooter/Top/TargetVelocity", topTargetRPM);

        Logger.recordOutput("Shooter/Bottom/CurrentDraw", bottomMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/Bottom/Temperature", bottomMotor.getMotorTemperature());
        Logger.recordOutput("Shooter/Bottom/Velocity", bottomEncoder.getVelocity());
        Logger.recordOutput("Shooter/Bottom/TargetVelocity", bottomTargetRPM);
    }

    @Override
    public void writePeriodic() {
        topController.setReference(topTargetRPM, CANSparkBase.ControlType.kVelocity);
        bottomController.setReference(bottomTargetRPM, CANSparkBase.ControlType.kVelocity);

        if (RobotState.isDisabled())
        {
            topController.setIAccum(0);
            bottomController.setIAccum(0);
        }
    }

    @Override
    public void setTopTargetRPM(double targetRPM) {
        this.topTargetRPM = targetRPM;
    }

    @Override
    public void setBottomTargetRPM(double targetRPM) {
        this.bottomTargetRPM = targetRPM;
    }


}
