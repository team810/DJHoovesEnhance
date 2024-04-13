package frc.robot.commands;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;


public class TrajectoryCommand extends Command {
    private final ChoreoTrajectory trajectory;
    private ChoreoTrajectoryState stateCurrent;

    private final Timer timer;

    @Override
    public void initialize() {
        timer.start();
        stateCurrent = trajectory.sample(0,DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);

    }

    @Override
    public void execute() {
        stateCurrent = trajectory.sample(timer.get(), DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);

    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        DrivetrainSubsystem.getInstance().setDrivetrainMode(DrivetrainSubsystem.DrivetrainMode.off);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTime());
    }

    public TrajectoryCommand(ChoreoTrajectory trajectory)
    {
        this.timer = new Timer();
        this.trajectory = trajectory;

        addRequirements(DrivetrainSubsystem.getInstance());
    }
}
