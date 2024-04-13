package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.littletonrobotics.junction.Logger;

public class IntakeSim implements IntakeIO {

    private final FlywheelSim topMotor;

    private final FlywheelSim bottomMotor;

    public IntakeSim() {

        topMotor = new FlywheelSim(DCMotor.getNEO(1), 1, 0.1);
        bottomMotor = new FlywheelSim(DCMotor.getNEO(1), 1, 0.1);

    }

    @Override
    public void setVoltage(double voltage) {
        topMotor.setInputVoltage(-voltage);
        bottomMotor.setInputVoltage(voltage);
    }

    @Override
    public void update() {
        Logger.recordOutput("Intake/Top/CurrentDraw", topMotor.getCurrentDrawAmps());
        Logger.recordOutput("Intake/Top/Velocity", topMotor.getAngularVelocityRPM());

        Logger.recordOutput("Intake/Bottom/CurrentDraw", bottomMotor.getCurrentDrawAmps());
        Logger.recordOutput("Intake/Bottom/Velocity", bottomMotor.getAngularVelocityRPM());
    }
}
