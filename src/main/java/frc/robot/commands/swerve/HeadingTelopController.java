package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class HeadingTelopController extends Command {
    private final SlewRateLimiter throttleLimiter;
    private final SlewRateLimiter thetaLimiter;

    private double invert = 1;

    public HeadingTelopController() {
        throttleLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ACCELERATION);
        thetaLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ANGULAR_ACCELERATION);
    }

    @Override
    public void execute() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            invert = -1;
        }else{
            invert = 1;
        }
        double xHeadingInput;
        double yHeadingInput;
        double thetaInput;
        double throttleInput;

        Rotation2d heading;

        xHeadingInput = -IO.getJoystickValue(Controls.headingX).get();
        yHeadingInput = -IO.getJoystickValue(Controls.headingY).get();
        throttleInput = IO.getJoystickValue(Controls.throttle).get();
        thetaInput = -IO.getJoystickValue(Controls.drive_theta).get();

        xHeadingInput = xHeadingInput * invert;
        yHeadingInput = yHeadingInput * invert;

        throttleInput = throttleLimiter.calculate(throttleInput);
        thetaInput = thetaLimiter.calculate(thetaInput);

        xHeadingInput = MathUtil.applyDeadband(xHeadingInput, .1);
        yHeadingInput = MathUtil.applyDeadband(yHeadingInput, .1);
        throttleInput = MathUtil.applyDeadband(throttleInput, .1);
        thetaInput = MathUtil.applyDeadband(thetaInput, .1);

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
        xVelocity = heading.getSin() * throttleInput;
        yVelocity = heading.getCos() * throttleInput;
        thetaVelocity = thetaInput;

        if (xHeadingInput == 0 && yHeadingInput == 0)
        {
            xVelocity = 0;
            yVelocity = 0;
        }

        DrivetrainSubsystem.getInstance().setTelopSpeeds(new ChassisSpeeds(xVelocity, yVelocity, thetaVelocity));

        rightStickAngle();
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

    void rightStickAngle()
    {
        double thetaX = -IO.getJoystickValue(Controls.thetaX).get();
        double thetaY = -IO.getJoystickValue(Controls.thetaY).get();

        boolean use;
        use = MathUtil.isNear(1,Math.hypot(thetaX, thetaY),.1);

        if (use)
        {
            DrivetrainSubsystem.getInstance().setTargetAngle(new Rotation2d(thetaY,thetaX));
        }else{
            DrivetrainSubsystem.getInstance().setTargetAngle(DrivetrainSubsystem.getInstance().getFiledRelativeOrientationOfRobot());
        }
    }
}