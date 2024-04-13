package frc.robot.subsystems.deflector;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public interface DeflectorIO {
    public void setState(DoubleSolenoid.Value value);
    public void update();
}
