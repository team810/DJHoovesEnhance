package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class TrajectoryHelper {

    public static Trajectory GenerateTrajectory(Targets target)
    {
        Pose2d startPose = DrivetrainSubsystem.getInstance().getRobotPose();

        return null;
    }

    public enum Targets
    {
        Speaker,
        Amp,
        Source,
    }
}
