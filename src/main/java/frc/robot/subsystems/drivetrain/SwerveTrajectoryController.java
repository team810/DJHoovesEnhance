package frc.robot.subsystems.drivetrain;

import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class SwerveTrajectoryController {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    public SwerveTrajectoryController(PIDController xController, PIDController yController, PIDController thetaController)
    {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;

        xController.setTolerance(.01);
        yController.setTolerance(.01);

        thetaController.setTolerance(Rotation2d.fromDegrees(1).getRadians());
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Shuffleboard.getTab("Swerve").add("xController", xController);
        Shuffleboard.getTab("Swerve").add("yController", yController);
        Shuffleboard.getTab("Swerve").add("thetaController", thetaController);
    }

    public ChassisSpeeds calculate(Pose2d currentPose, ChoreoTrajectoryState trajectoryState)
    {
        double xFF, yFF, thetaFF;
        double xFeedback, yFeedback, thetaFeedback;

        xFF = trajectoryState.velocityX;
        yFF = trajectoryState.velocityY;
        thetaFF = trajectoryState.angularVelocity;

        xFeedback = 0;
        yFeedback = 0;
        thetaFeedback = 0;

        xFeedback = xController.calculate(currentPose.getX(), trajectoryState.x);
        yFeedback = yController.calculate(currentPose.getY(), trajectoryState.y);
        thetaFeedback = thetaController.calculate(currentPose.getRotation().getRadians(), trajectoryState.heading);

        return ChassisSpeeds.fromFieldRelativeSpeeds(xFF + xFeedback, yFF + yFeedback, thetaFF + -thetaFeedback, currentPose.getRotation());
    }

    public ChassisSpeeds calculate(Pose2d currentPose, double xVelocity,double yVelocity,double thetaVelocity, double x, double y, double heading)
    {
        return this.calculate(currentPose, new ChoreoTrajectoryState(0,xVelocity,yVelocity, thetaVelocity, x,y,heading));
    }

    public boolean atReference()
    {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }
}
