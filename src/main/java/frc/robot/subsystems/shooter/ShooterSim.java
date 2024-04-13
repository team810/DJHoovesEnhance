package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class ShooterSim implements ShooterIO{

    private final FlywheelSim topMotor;
    private final FlywheelSim bottomMotor;

    private double topVoltage;
    private double bottomVoltage;



    public ShooterSim()
    {
        topMotor = new FlywheelSim(DCMotor.getNEO(1), 1, 0.1);
        bottomMotor = new FlywheelSim(DCMotor.getNEO(1), 1, 0.1);
    }

    @Override
    public void setTopTargetRPM(double targetRPM) {

    }

    @Override
    public void setBottomTargetRPM(double targetRPM) {

    }

    @Override
    public void readPeriodic() {
        topMotor.update(Robot.defaultPeriodSecs);
        bottomMotor.update(Robot.defaultPeriodSecs);
    }

    @Override
    public void writePeriodic() {
        Logger.recordOutput("Shooter/Top/Voltage", topVoltage);
        Logger.recordOutput("Shooter/Top/CurrentDraw", topMotor.getCurrentDrawAmps());
        Logger.recordOutput("Shooter/Top/Velocity", topMotor.getAngularVelocityRPM());

        Logger.recordOutput("Shooter/Bottom/Voltage", bottomVoltage);
        Logger.recordOutput("Shooter/Bottom/CurrentDraw", bottomMotor.getCurrentDrawAmps());
        Logger.recordOutput("Shooter/Bottom/Velocity", bottomMotor.getAngularVelocityRPM());
    }
}
