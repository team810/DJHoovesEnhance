package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.tbone.TBoneSubsystem;


public class TBoneCommand extends Command {

    public TBoneCommand() {
        addRequirements(TBoneSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        TBoneSubsystem.getInstance().toggleState();
    }
}
