package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.laser.LaserState;
import frc.robot.subsystems.laser.LaserSubsystem;

public class IntakeAutoCommand extends Command {
    public IntakeAutoCommand() {
        addRequirements(IntakeSubsystem.getInstance(), LaserSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        IntakeSubsystem.getInstance().setState(IntakeStates.fwd);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setState(IntakeStates.off);
    }

    @Override
    public boolean isFinished() {
        if (Robot.isReal())
        {
            return LaserSubsystem.getInstance().getLaserState() == LaserState.Detected;
        }else{
            return false;
        }
    }
}
