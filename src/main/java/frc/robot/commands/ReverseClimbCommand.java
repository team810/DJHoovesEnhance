package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberStates;
import frc.robot.subsystems.climber.ClimberSubsystem;


public class ReverseClimbCommand extends Command {

    public ReverseClimbCommand() {

        addRequirements(ClimberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ClimberSubsystem.getInstance().setState(ClimberStates.up);
    }


    @Override
    public void end(boolean interrupted) {
        ClimberSubsystem.getInstance().setState(ClimberStates.off);
    }
}
