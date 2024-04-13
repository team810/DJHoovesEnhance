package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.MechanismState;
import frc.robot.subsystems.deflector.DeflectorSubsystem;
import frc.robot.subsystems.shooter.ShooterMode;
import frc.robot.subsystems.shooter.ShooterSubsystem;


public class RevTapeCommand extends Command {

    public RevTapeCommand() {

        addRequirements(ShooterSubsystem.getInstance(), DeflectorSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ShooterSubsystem.getInstance().setShooterMode(ShooterMode.Tape);
        DeflectorSubsystem.getInstance().setDeflectorState(MechanismState.stored);
    }



    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setShooterMode(ShooterMode.off);
        DeflectorSubsystem.getInstance().setDeflectorState(MechanismState.stored);
    }
}
