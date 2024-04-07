package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class TelopController extends Command {
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter thetaLimiter;

    public TelopController() {
        xLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ACCELERATION);
        yLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ACCELERATION);
        thetaLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ANGULAR_ACCELERATION);

        addRequirements(DrivetrainSubsystem.getInstance());
    }

    @Override
    public void execute() {
        double xInput = -IO.getJoystickValue(Controls.drive_x).get();
        double yInput = -IO.getJoystickValue(Controls.drive_y).get();
        double thetaInput = -IO.getJoystickValue(Controls.drive_theta).get();

        xInput = xLimiter.calculate(xInput);
        yInput = yLimiter.calculate(yInput);

        Translation2d linerVelocityCalc = calcLinearVelocity(xInput, yInput);

        thetaInput = thetaLimiter.calculate(thetaInput);
        thetaInput = MathUtil.applyDeadband(thetaInput, .05);
        thetaInput = Math.pow(thetaInput, 3);


        if (DrivetrainSubsystem.getInstance().getSpeedMode() == DrivetrainSubsystem.SpeedMode.normal)
        {
            xInput = linerVelocityCalc.getX() * DrivetrainConstants.NORMAL_SPEED;
            yInput = linerVelocityCalc.getY() * DrivetrainConstants.NORMAL_SPEED;
            thetaInput = thetaInput * DrivetrainConstants.NORMAL_ANGULAR_VELOCITY;

        } else if (DrivetrainSubsystem.getInstance().getSpeedMode() == DrivetrainSubsystem.SpeedMode.slow) {
            xInput = linerVelocityCalc.getX() * DrivetrainConstants.SLOW_SPEED;
            yInput = linerVelocityCalc.getY() * DrivetrainConstants.SLOW_SPEED;
            thetaInput = thetaInput * DrivetrainConstants.SLOW_ANGULAR_VELOCITY;
        }
        DrivetrainSubsystem.getInstance().setTelopSpeeds(
                new ChassisSpeeds(
                    yInput,
                    xInput,
                    thetaInput
                ));
    }
    public static Translation2d calcLinearVelocity(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), .1);
        Rotation2d linearDirection = new Rotation2d(x, y);

        // Square magnitude
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Calcaulate new linear velocity
        Translation2d linearVelocity =
                new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                        .getTranslation();
        return linearVelocity;
    }

    @Override
    public void initialize() {
        DrivetrainSubsystem.getInstance().setDrivetrainMode(DrivetrainSubsystem.DrivetrainMode.telop);
    }

    @Override
    public void end(boolean interrupted) {DrivetrainSubsystem.getInstance().setDrivetrainMode(DrivetrainSubsystem.DrivetrainMode.off);}

    @Override
    public boolean isFinished() {return !RobotState.isTeleop();}
}
