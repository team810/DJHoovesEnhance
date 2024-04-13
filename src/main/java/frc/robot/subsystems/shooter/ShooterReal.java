package frc.robot.subsystems.shooter;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.RobotState;
import org.littletonrobotics.junction.Logger;

public class ShooterReal implements ShooterIO {
    private final CANSparkMax topMotor;
    private final CANSparkMax bottomMotor;

    private final RelativeEncoder topEncoder;
    private final RelativeEncoder bottomEncoder;

    private double topTargetRPM;
    private double bottomTargetRPM;

    private final SparkPIDController topController;
    private final SparkPIDController bottomController;

    public double BottomP, BottomI, BottomD, BottomIz, BottomFF, BottomMaxOutput,BottomMinOutput, BottomMaxRPM;
    public double TopP, TopI, TopD, TopIz, TopFF, TopMaxOutput, TopMinOutput, TopMaxRPM;

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

//
//        SmartDashboard.putNumber("Top/kP", 0.00006);
//        SmartDashboard.putNumber("Top/kI", TopI);
//        SmartDashboard.putNumber("Top/kD", TopD);
//        SmartDashboard.putNumber("Top/kFF", 0.000185);
//
//        SmartDashboard.putNumber("Bottom/kP", 0.00006);
//        SmartDashboard.putNumber("Bottom/kI", BottomI);
//        SmartDashboard.putNumber("Bottom/kD", BottomD);
//        SmartDashboard.putNumber("Bottom/kFF", 0.000185);

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
    public void update()
    {
//        TopP = SmartDashboard.getNumber("Top/kP",0);
//        TopI = SmartDashboard.getNumber("Top/kI",0);
//        TopD = SmartDashboard.getNumber("Top/kD", 0);
//        TopFF = SmartDashboard.getNumber("Top/kFF", 0);
//
//        BottomP = SmartDashboard.getNumber("Bottom/kP",0);
//        BottomI = SmartDashboard.getNumber("Bottom/kI",0);
//        BottomD = SmartDashboard.getNumber("Bottom/kD", 0);
//        BottomFF = SmartDashboard.getNumber("Bottom/kFF", 0);



        Logger.recordOutput("Shooter/Top/CurrentDraw", topMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/Top/Temperature", topMotor.getMotorTemperature());
        Logger.recordOutput("Shooter/Top/Velocity", topEncoder.getVelocity());
        Logger.recordOutput("Shooter/Top/TargetVelocity", topTargetRPM);

        Logger.recordOutput("Shooter/Bottom/CurrentDraw", bottomMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/Bottom/Temperature", bottomMotor.getMotorTemperature());
        Logger.recordOutput("Shooter/Bottom/Velocity", bottomEncoder.getVelocity());
        Logger.recordOutput("Shooter/Bottom/TargetVelocity", bottomTargetRPM);


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
