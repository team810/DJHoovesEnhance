package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.MechanismState;
import frc.robot.subsystems.deflector.DeflectorSubsystem;
import frc.robot.subsystems.shooter.ShooterMode;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.tbone.TBoneSubsystem;


public class AmpScoreCommand extends Command {

    public AmpScoreCommand() {
        addRequirements(
                ShooterSubsystem.getInstance(),
                DeflectorSubsystem.getInstance(),
                TBoneSubsystem.getInstance()
        );
    }

    @Override
    public void initialize() {
        TBoneSubsystem.getInstance().setState(MechanismState.deployed);
        ShooterSubsystem.getInstance().setShooterMode(ShooterMode.Amp);
    }

    @Override
    public void end(boolean interrupted) {
        TBoneSubsystem.getInstance().setState(MechanismState.stored);
        ShooterSubsystem.getInstance().setShooterMode(ShooterMode.off);
    }
}
