package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeStates;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class FireCommand extends Command {

    public FireCommand() {

        addRequirements(IntakeSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        IntakeSubsystem.getInstance().setState(IntakeStates.fire);
    }
    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setState(IntakeStates.off);
    }
}
