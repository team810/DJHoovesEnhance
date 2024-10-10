package frc.robot.subsystems.drivetrain;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class SwerveTrajectoryController {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    private final boolean useFeedback;

    public SwerveTrajectoryController(PIDController xController, PIDController yController, PIDController thetaController, boolean useFeedback)
    {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;

        this.useFeedback = useFeedback;

        xController.setTolerance(.01);
        yController.setTolerance(.01);

        thetaController.setTolerance(Rotation2d.fromDegrees(1).getRadians());
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Shuffleboard.getTab("Swerve").add("xController", xController);
        Shuffleboard.getTab("Swerve").add("yController", yController);
        Shuffleboard.getTab("Swerve").add("thetaController", thetaController);
    }

//    public ChassisSpeeds calculate(Pose2d currentPose, ChoreoTrajectoryState trajectoryState)
//    {
//        double xFF, yFF, thetaFF;
//        double xFeedback, yFeedback, thetaFeedback;
//
//        xFF = 0;
//        yFF = 0;
//        thetaFF = 0;
//
//        xFeedback = 0;
//        yFeedback = 0;
//        thetaFeedback = 0;
//
//        if (useFeedback)
//        {
//            xFeedback = xController.calculate(currentPose.getX(), trajectoryState.x);
//            yFeedback = yController.calculate(currentPose.getY(), trajectoryState.y);
//            thetaFeedback = thetaController.calculate(currentPose.getRotation().getRadians(), trajectoryState.getPose().getRotation().getRadians());
//        }
//
//        return ChassisSpeeds.fromFieldRelativeSpeeds(xFF + xFeedback, yFF + yFeedback, thetaFF + thetaFeedback, currentPose.getRotation());
//    }

    public boolean atReference()
    {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }
}
