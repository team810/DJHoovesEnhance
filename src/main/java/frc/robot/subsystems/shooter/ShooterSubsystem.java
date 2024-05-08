package frc.robot.subsystems.shooter;


import frc.robot.Robot;
import frc.robot.lib.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends AdvancedSubsystem {
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
    public void readPeriodic() {
        shooter.readPeriodic();
    }

    @Override
    public void writePeriodic() {
        Logger.recordOutput("Shooter/Top/TargetSpeedSub", topTargetSpeed);
        Logger.recordOutput("Shooter/Bottom/TargetSpeedSub", bottomTargetSpeed);
        Logger.recordOutput("Shooter/Mode/ShooterMode", shooterMode);

        shooter.writePeriodic();
    }

    public void setShooterMode(ShooterMode shooterMode) {
        this.shooterMode = shooterMode;

        switch (shooterMode)
        {
            case SourceIntake -> {
                topTargetSpeed = -2000;
                bottomTargetSpeed = -2000;
            }
            case Amp -> {
                topTargetSpeed = 2500;
                bottomTargetSpeed = 2500;
            }
            case Tape -> {
                topTargetSpeed = 4000;
                bottomTargetSpeed = 2000;
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

        shooter.setTopTargetRPM(topTargetSpeed);
        shooter.setBottomTargetRPM(bottomTargetSpeed);
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


