package frc.robot.commands;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;


public class TrajectoryCommand extends Command {
    private final ChoreoTrajectory trajectory;

    private ChoreoTrajectoryState stateCurrent;

    private Trajectory.State currentTrajectoryState;
    private Trajectory.State previousTrajectoryState;

    private double previousTimestamp;

    private final Timer timer;

    public TrajectoryCommand(ChoreoTrajectory trajectory)
    {
        this.timer = new Timer();
        this.trajectory = trajectory;

        addRequirements(DrivetrainSubsystem.getInstance());
    }
    @Override
    public void initialize() {
        DrivetrainSubsystem.getInstance().setDrivetrainMode(DrivetrainSubsystem.DrivetrainMode.trajectory);

        timer.start();
        stateCurrent = trajectory.sample(0,DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);

        double velocity = Math.sqrt(
                Math.pow(stateCurrent.velocityX,2) + Math.pow(stateCurrent.velocityY, 2)
        );
        double acceleration = velocity;
        double curvature = stateCurrent.angularVelocity/velocity;

        currentTrajectoryState = new Trajectory.State(
                stateCurrent.timestamp,
                velocity,
                acceleration,
                stateCurrent.getPose(),
                curvature
        );

        DrivetrainSubsystem.getInstance().setTargetTrajectoryState(currentTrajectoryState);

        previousTrajectoryState = currentTrajectoryState;
        previousTimestamp = timer.get();
    }

    @Override
    public void execute() {
        double delta = timer.get() - previousTimestamp;
        stateCurrent = trajectory.sample(timer.get(), DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);

        double velocity = Math.sqrt(
                Math.pow(stateCurrent.velocityX,2) + Math.pow(stateCurrent.velocityY, 2)
        );
        double acceleration = (velocity - previousTrajectoryState.velocityMetersPerSecond) / delta;
        double curvature = stateCurrent.angularVelocity/velocity;

        currentTrajectoryState = new Trajectory.State(
                stateCurrent.timestamp,
                velocity,
                acceleration,
                stateCurrent.getPose(),
                curvature
        );
        DrivetrainSubsystem.getInstance().setTargetTrajectoryState(currentTrajectoryState);
        previousTrajectoryState = currentTrajectoryState;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTime()) && DrivetrainSubsystem.getInstance().trajectoryControllerAtSetpoint();
    }
}
