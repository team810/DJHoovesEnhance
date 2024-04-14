package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.ChoreoTrajectoryCommand;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class AutoBuilder {
    public static Command getAuto()
    {
        ChoreoTrajectory trajectory = Choreo.getTrajectory("4 Peice");

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
                    {
                        DrivetrainSubsystem.getInstance().setFiledRelativePose(Choreo.getTrajectory("Testing").flipped().getInitialPose());
                    }else{
                        DrivetrainSubsystem.getInstance().setFiledRelativePose(Choreo.getTrajectory("Testing").getInitialPose());
                    }
                },DrivetrainSubsystem.getInstance()),
                new ChoreoTrajectoryCommand(trajectory)
        );
    }
}
