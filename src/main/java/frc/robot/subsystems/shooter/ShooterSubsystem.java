package frc.robot.subsystems.shooter;


import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem INSTANCE;

    private final ShooterIO shooter;

    private double topTargetSpeed;
    private double bottomTargetSpeed;

    private ShooterMode shooterMode;

    private double targetTopTestRPM;
    private double targetBottomTestRPM;

    private ShooterSubsystem()
    {
        if (Robot.isReal())
        {
            shooter = new ShooterReal();

        }else{
            shooter = new ShooterSim();
        }

        topTargetSpeed = 0;
        bottomTargetSpeed = 0;

        shooterMode = ShooterMode.off;

        targetTopTestRPM = 2000;
        targetBottomTestRPM = 2000;
    }

    @Override
    public void periodic() {
        if (RobotState.isEnabled())
        {
            switch (shooterMode)
            {
                case SourceIntake -> {
                    topTargetSpeed = -2000;
                    bottomTargetSpeed = -2000;
                }
                case Amp -> {
                    topTargetSpeed = 1800;
                    bottomTargetSpeed = 1800;
                }
                case Tape -> {
                    topTargetSpeed = targetTopTestRPM;
                    bottomTargetSpeed = targetBottomTestRPM;
                }
                case Subwoofer -> {
                    topTargetSpeed = 3500;
                    bottomTargetSpeed = 3500;
                }
                case test -> {
                    topTargetSpeed = targetTopTestRPM;
                    bottomTargetSpeed = targetBottomTestRPM;
                }
                case off -> {
                    topTargetSpeed = 0;
                    bottomTargetSpeed = 0;
                }
            }
        }else{
            topTargetSpeed = 0;
            bottomTargetSpeed = 0;
        }

        shooter.setTopTargetRPM(topTargetSpeed);
        shooter.setBottomTargetRPM(bottomTargetSpeed);

        Logger.recordOutput("Shooter/Top/TargetSpeedSub", topTargetSpeed);
        Logger.recordOutput("Shooter/Bottom/TargetSpeedSub", bottomTargetSpeed);
        Logger.recordOutput("Shooter/Mode/ShooterMode", shooterMode);

        shooter.update();
    }

    public void setShooterMode(ShooterMode shooterMode) {
        this.shooterMode = shooterMode;
    }


    public static ShooterSubsystem getInstance()
    {
        if (INSTANCE == null)
        {
            INSTANCE = new ShooterSubsystem();
        }
        return INSTANCE;
    }
}


