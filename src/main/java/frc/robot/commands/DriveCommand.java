package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class DriveCommand extends Command {
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter thetaLimiter;

    public DriveCommand() {
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
        thetaInput = thetaLimiter.calculate(thetaInput);

        xInput = MathUtil.applyDeadband(xInput,.05);
        yInput = MathUtil.applyDeadband(yInput,.05);
        thetaInput = MathUtil.applyDeadband(thetaInput, .05);

        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        thetaInput = Math.pow(thetaInput, 3);


        if (DrivetrainSubsystem.getInstance().getSpeedMode() == DrivetrainSubsystem.SpeedMode.normal)
        {
            xInput = xInput * DrivetrainConstants.NORMAL_SPEED;
            yInput = yInput * DrivetrainConstants.NORMAL_SPEED;
            thetaInput = thetaInput * DrivetrainConstants.NORMAL_ANGULAR_VELOCITY;

        } else if (DrivetrainSubsystem.getInstance().getSpeedMode() == DrivetrainSubsystem.SpeedMode.slow) {
            xInput = xInput * DrivetrainConstants.SLOW_SPEED;
            yInput = yInput * DrivetrainConstants.SLOW_SPEED;
            thetaInput = thetaInput * DrivetrainConstants.SLOW_ANGULAR_VELOCITY;
        }
        DrivetrainSubsystem.getInstance().setTelopSpeeds(
                new ChassisSpeeds(
                    yInput,
                    xInput,
                    thetaInput
                ));
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
