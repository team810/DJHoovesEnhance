package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterMode;
import frc.robot.subsystems.shooter.ShooterSubsystem;


public class RevShooterTestCommand extends Command {

    public RevShooterTestCommand() {
        addRequirements(ShooterSubsystem.getInstance());
    }

    @Override
    public void initialize() {
       ShooterSubsystem.getInstance().setShooterMode(ShooterMode.test);
    }

    @Override
    public void end(boolean interrupted) {
       ShooterSubsystem.getInstance().setShooterMode(ShooterMode.off);
    }
}
