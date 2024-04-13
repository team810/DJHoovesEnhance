package frc.robot.subsystems.tbone;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.lib.AdvancedSubsystem;
import frc.robot.lib.MechanismState;
import org.littletonrobotics.junction.Logger;

public class TBoneSubsystem extends AdvancedSubsystem {
    private static TBoneSubsystem INSTANCE;
    private final TBoneIO tBone;

    private MechanismState state;
    private final PIDController controller = new PIDController(4,0.1,0);

    private double setpoint;

    private TBoneSubsystem() {
        tBone = new TboneReal();

        controller.setTolerance(0);
        setState(MechanismState.stored);
    }

    @Override
    public void readPeriodic() {
        tBone.readPeriodic();
    }

    @Override
    public void writePeriodic() {
        if (RobotState.isEnabled())
        {
            if (state == MechanismState.stored)
            {
                tBone.setVoltage(
                        MathUtil.clamp(
                                controller.calculate(tBone.getEncoderPosition(), setpoint),-.5,.5
                        )
                );
            } else if (state == MechanismState.deployed)
            {
                tBone.setVoltage(
                        MathUtil.clamp(
                                controller.calculate(tBone.getEncoderPosition(), setpoint),-6,6
                        )
                );
            }
        }

        Logger.recordOutput("T-Bone/Setpoint", setpoint);
        Logger.recordOutput("T-Bone/AtSetpoint", controller.atSetpoint());
        tBone.writePeriodic();
    }

    public MechanismState getState() {
        return state;
    }

    public void setState(MechanismState state) {
        this.state = state;
        switch (getState())
        {
            case deployed -> {
                setpoint = TboneConstants.DEPLOY_SETPOINT;
            }
            case stored -> {
                setpoint = TboneConstants.STORED_SETPOINT;
            }
        }
    }

    public void toggleState() {
        if (this.state == MechanismState.deployed) {
            setState(MechanismState.stored);
        } else {
            setState(MechanismState.deployed);
        }
    }
    public static TBoneSubsystem getInstance() {
        if (INSTANCE == null) { INSTANCE = new TBoneSubsystem(); }
        return INSTANCE;
    }
}

