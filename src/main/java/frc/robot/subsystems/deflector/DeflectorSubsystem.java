package frc.robot.subsystems.deflector;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Robot;
import frc.robot.lib.AdvancedSubsystem;
import frc.robot.lib.MechanismState;
import org.littletonrobotics.junction.Logger;

public class DeflectorSubsystem extends AdvancedSubsystem {

    private static DeflectorSubsystem INSTANCE;
    private final DeflectorIO deflector;
    private MechanismState deflectorState;

    private DeflectorSubsystem() {
        deflector = new DeflectorReal();

        deflectorState = MechanismState.stored;
        deflector.setState(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void readPeriodic() {

    }

    @Override
    public void writePeriodic() {
        Logger.recordOutput("Deflector/MechanismState", deflectorState);
        deflector.writePeriodic();
    }

    public MechanismState getDeflectorState() {
        return deflectorState;
    }

    public void setDeflectorState(MechanismState deflectorState) {
        this.deflectorState = deflectorState;
        if (deflectorState == MechanismState.deployed) {
            deflector.setState(DoubleSolenoid.Value.kForward);
        } else if (deflectorState == MechanismState.stored) {
            deflector.setState(DoubleSolenoid.Value.kReverse);
        }
    }

    public void toggleDeflectorState()
    {
        if (deflectorState == MechanismState.deployed) {
            deflector.setState(DoubleSolenoid.Value.kForward);
            deflectorState = MechanismState.stored;
        } else if (deflectorState == MechanismState.stored) {
            deflector.setState(DoubleSolenoid.Value.kReverse);
            deflectorState = MechanismState.deployed;
        }
    }

    public static DeflectorSubsystem getInstance() {
        if (INSTANCE == null)
        {
            INSTANCE = new DeflectorSubsystem();
        }
        return INSTANCE;
    }
}

