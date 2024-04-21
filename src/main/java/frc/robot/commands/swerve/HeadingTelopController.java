package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

class HeadingTelopController extends Command {
    private final SlewRateLimiter throttleLimiter;
    private final SlewRateLimiter thetaLimiter;

    public HeadingTelopController() {
        throttleLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ACCELERATION);
        thetaLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ANGULAR_ACCELERATION);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds;
        Rotation2d heading;

        double xHeadingInput;
        double yHeadingInput;
        double thetaInput;
        double throttleInput;

        xHeadingInput = IO.getJoystickValue(Controls.headingX).get();
        yHeadingInput = IO.getJoystickValue(Controls.headingY).get();
        throttleInput = IO.getJoystickValue(Controls.throttle).get();
        thetaInput = IO.getJoystickValue(Controls.drive_theta).get();

        throttleInput = throttleLimiter.calculate(throttleInput);
        thetaInput = thetaLimiter.calculate(thetaInput);

        xHeadingInput = MathUtil.applyDeadband(xHeadingInput, .05);
        yHeadingInput = MathUtil.applyDeadband(yHeadingInput, .05);
        throttleInput = MathUtil.applyDeadband(throttleInput, .05);
        thetaInput = MathUtil.applyDeadband(thetaInput, .05);

        thetaInput = Math.copySign(thetaInput * thetaInput, thetaInput);
        throttleInput = Math.pow(throttleInput, 3);

        heading = new Rotation2d(xHeadingInput, yHeadingInput);

        if (DrivetrainSubsystem.getInstance().getSpeedMode() == DrivetrainSubsystem.SpeedMode.normal)
        {
            throttleInput = throttleInput * DrivetrainConstants.NORMAL_SPEED;
            thetaInput = thetaInput * DrivetrainConstants.NORMAL_ANGULAR_VELOCITY;
        } else if (DrivetrainSubsystem.getInstance().getSpeedMode() == DrivetrainSubsystem.SpeedMode.slow) {
            throttleInput = throttleInput * DrivetrainConstants.SLOW_SPEED;
            thetaInput = thetaInput * DrivetrainConstants.SLOW_ANGULAR_VELOCITY;
        }

        double xVelocity;
        double yVelocity;
        double thetaVelocity;

        xVelocity = heading.getCos() * throttleInput;
        yVelocity = heading.getSin() * throttleInput;
        thetaVelocity = thetaInput;

        DrivetrainSubsystem.getInstance().setTelopSpeeds(new ChassisSpeeds(xVelocity, yVelocity, thetaVelocity));
    }

    @Override
    public void initialize() {
        DrivetrainSubsystem.getInstance().setDrivetrainMode(DrivetrainSubsystem.DrivetrainMode.telop);
    }

    @Override
    public void end(boolean interrupted) {
        DrivetrainSubsystem.getInstance().setDrivetrainMode(DrivetrainSubsystem.DrivetrainMode.off);
    }

    @Override
    public boolean isFinished() {
        return !RobotState.isTeleop();
    }
}
