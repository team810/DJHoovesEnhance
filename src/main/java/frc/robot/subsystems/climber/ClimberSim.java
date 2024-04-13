package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.littletonrobotics.junction.Logger;

public class ClimberSim implements ClimberIO {

    private final FlywheelSim climberMotor;
    private double inputVoltage;

    public ClimberSim() {
        climberMotor = new FlywheelSim(DCMotor.getNEO(1), 100, 1);
    }

    @Override
    public void setVoltage(double voltage) {
        this.inputVoltage = voltage;
        climberMotor.setInputVoltage(voltage);
    }

    @Override
    public void update() {
        Logger.recordOutput("Climber/currentDraw", climberMotor.getCurrentDrawAmps());
        Logger.recordOutput("Climber/Velocity", climberMotor.getAngularVelocityRPM());
        Logger.recordOutput("Climber/inputVoltage", this.inputVoltage);
        
    }

    @Override
    public void release() {
        
    }

    public void pin() { }


}
