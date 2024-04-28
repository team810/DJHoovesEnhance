package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IO.IO;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class DPadTurn extends Command {
    public DPadTurn() {}

    @Override
    public void initialize() {
        DrivetrainSubsystem.getInstance().setHeadingControlMode(DrivetrainSubsystem.HeadingControlMode.dpad);
        execute();
    }

    @Override
    public void execute() {
        double input = IO.getDPadPrimary();
        if (input != -1) {
            Rotation2d rot = Rotation2d.fromDegrees(input);
            rot = rot.unaryMinus();
            rot = Rotation2d.fromRadians(MathUtil.angleModulus(rot.getRadians()));
            DrivetrainSubsystem.getInstance().setTargetAngle(rot);
        }else{
            DrivetrainSubsystem.getInstance().setHeadingControlMode(DrivetrainSubsystem.HeadingControlMode.velocity);
        }
    }

    @Override
    public boolean isFinished() {
        return IO.getDPadPrimary() == -1;
    }

    @Override
    public void end(boolean interrupted) {
        DrivetrainSubsystem.getInstance().setHeadingControlMode(DrivetrainSubsystem.HeadingControlMode.velocity);
        DrivetrainSubsystem.getInstance().setDrivetrainMode(DrivetrainSubsystem.DrivetrainMode.telop);
    }
}
