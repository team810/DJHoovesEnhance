package frc.robot.subsystems.deflector;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.util.Pneumatics;
import org.littletonrobotics.junction.Logger;

public class DeflectorReal implements DeflectorIO{
    private final DoubleSolenoid deflector;

    public DeflectorReal()
    {
        deflector = Pneumatics.getInstance().createSolenoid(DeflectorConstants.DEFLECTOR_FWD_CHANNEL, DeflectorConstants.DEFLECTOR_REV_CHANNEL);
    }
    @Override
    public void setState(DoubleSolenoid.Value value) {
        deflector.set(value);
    }

    @Override
    public void writePeriodic() {
        Logger.recordOutput("Deflector/SolenoidValue", deflector.get());
    }
}
