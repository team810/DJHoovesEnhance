package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.MechanismState;
import frc.robot.subsystems.deflector.DeflectorSubsystem;
import frc.robot.subsystems.intake.IntakeStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.laser.LaserState;
import frc.robot.subsystems.laser.LaserSubsystem;
import frc.robot.subsystems.shooter.ShooterMode;
import frc.robot.subsystems.shooter.ShooterSubsystem;


public class IntakeSourceCommand extends Command {

    public IntakeSourceCommand() {
        addRequirements(ShooterSubsystem.getInstance(), IntakeSubsystem.getInstance(), DeflectorSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        DeflectorSubsystem.getInstance().setDeflectorState(MechanismState.deployed);
        IntakeSubsystem.getInstance().setState(IntakeStates.rev);
        ShooterSubsystem.getInstance().setShooterMode(ShooterMode.SourceIntake);
    }

    @Override
    public boolean isFinished() {
        return LaserSubsystem.getInstance().getLaserState() == LaserState.Detected;
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setState(IntakeStates.off);
        DeflectorSubsystem.getInstance().setDeflectorState(MechanismState.stored);
        ShooterSubsystem.getInstance().setShooterMode(ShooterMode.off);
    }
}
