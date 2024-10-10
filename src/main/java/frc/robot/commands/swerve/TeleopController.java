package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class TeleopController extends Command {
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter thetaLimiter;

    private double invert = 1;

    public TeleopController() {
        xLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ACCELERATION);
        yLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ACCELERATION);
        thetaLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ANGULAR_ACCELERATION);
    }


    @Override
    public void execute() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            invert = -1;
        }else{
            invert = 1;
        }

        double xInput = -IO.getJoystickValue(Controls.drive_x).get();
        double yInput = -IO.getJoystickValue(Controls.drive_y).get();
        double thetaInput = -IO.getJoystickValue(Controls.drive_theta).get();

        xInput = xInput * invert;
        yInput = yInput * invert;

        xInput = xLimiter.calculate(xInput);
        yInput = yLimiter.calculate(yInput);
        thetaInput = thetaLimiter.calculate(thetaInput);

        Translation2d linerVelocityCalc = calcLinearVelocity(xInput, yInput);

        thetaInput = MathUtil.applyDeadband(thetaInput, .1);
        thetaInput = Math.copySign(thetaInput * thetaInput, thetaInput);

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
        DrivetrainSubsystem.getInstance().setTeleopSpeeds(
               new ChassisSpeeds(
                    yInput,
                    xInput,
                    thetaInput
                ));

        double thetaX = IO.getJoystickValue(Controls.thetaX).get();
        double thetaY = IO.getJoystickValue(Controls.thetaY).get();

        boolean use;

        use = MathUtil.isNear(1,Math.hypot(thetaX, thetaY),.25);

        Rotation2d rawRotation = new Rotation2d(-thetaY, -thetaX);
        Rotation2d outputRotation = new Rotation2d();

        double tolerance = 22.5;
        if (use)
        {
            if (MathUtil.isNear(0, rawRotation.getDegrees(),tolerance)) {
                outputRotation = Rotation2d.fromDegrees(0);
            } else if (MathUtil.isNear(45, rawRotation.getDegrees(),tolerance)) {
                outputRotation = Rotation2d.fromDegrees(45);
            } else if (MathUtil.isNear(90, rawRotation.getDegrees(),tolerance)) {
                outputRotation = Rotation2d.fromDegrees(90);
            } else if (MathUtil.isNear(135, rawRotation.getDegrees(),tolerance)) {
                outputRotation = Rotation2d.fromDegrees(135);
            } else if (MathUtil.isNear(180, rawRotation.getDegrees(),tolerance) || MathUtil.isNear(-180, rawRotation.getDegrees(),tolerance)) {
                outputRotation = Rotation2d.fromDegrees(180);
            } else if (MathUtil.isNear(-135, rawRotation.getDegrees(),tolerance)) {
                outputRotation = Rotation2d.fromDegrees(-135);
            } else if (MathUtil.isNear(-135, rawRotation.getDegrees(),tolerance)) {
                outputRotation = Rotation2d.fromDegrees(-135);
            } else if (MathUtil.isNear(-90, rawRotation.getDegrees(),tolerance)) {
                outputRotation = Rotation2d.fromDegrees(-90);
            } else if (MathUtil.isNear(-45, rawRotation.getDegrees(),tolerance)) {
                outputRotation = Rotation2d.fromDegrees(-45);
            }
            DrivetrainSubsystem.getInstance().setTargetAngle(outputRotation);
        }else{
            DrivetrainSubsystem.getInstance().setTargetAngle(DrivetrainSubsystem.getInstance().getFiledRelativeOrientationOfRobot());
        }
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
        DrivetrainSubsystem.getInstance().setDrivetrainMode(DrivetrainSubsystem.DrivetrainMode.teleop);
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            invert = -1;
        }
    }

    @Override
    public void end(boolean interrupted) {
        DrivetrainSubsystem.getInstance().setDrivetrainMode(DrivetrainSubsystem.DrivetrainMode.off);
        System.out.println(interrupted);
    }

    @Override
    public boolean isFinished() {return !RobotState.isTeleop();}
}
