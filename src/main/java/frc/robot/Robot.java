
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.commands.*;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.deflector.DeflectorSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.laser.LaserSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.tbone.TBoneSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot
{
    private Command autonomousCommand;

    @Override
    public void robotInit() {
        Logger.recordMetadata("ProjectName", "Swerve Test"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }
        Logger.start();

        IO.Initialize();
        DriverStation.silenceJoystickConnectionWarning(true);

        DrivetrainSubsystem.getInstance();
        ShooterSubsystem.getInstance();
        IntakeSubsystem.getInstance();
        ClimberSubsystem.getInstance();
        TBoneSubsystem.getInstance();
        LaserSubsystem.getInstance();

        setUseTiming(true);
        CommandScheduler.getInstance().unregisterAllSubsystems();
        CommandScheduler.getInstance().setPeriod(.15);

        autonomousCommand = AutoBuilder.getAuto();

        new Trigger(() -> IO.getButtonValue(Controls.reset_gyro).get()).toggleOnTrue(new InstantCommand(() -> DrivetrainSubsystem.getInstance().resetGyroButton()));

        new Trigger(() -> IO.getButtonValue(Controls.intakeFWD).get()).whileTrue(new IntakeFwdCommand());
        new Trigger(() -> IO.getButtonValue(Controls.intakeREVS).get()).whileTrue(new IntakeRevCommand());
        new Trigger(() -> IO.getButtonValue(Controls.sourceIntake).get()).whileTrue(new IntakeSourceCommand());

        new Trigger(() -> IO.getButtonValue(Controls.fire).get()).whileTrue(new FireCommand());
//        new Trigger(() -> IO.getButtonValue(Controls.SpeakerScore).get()).whileTrue(new RevShooterTestCommand());

        new Trigger(() -> IO.getButtonValue(Controls.AmpScore).get()).whileTrue(new AmpScoreCommand());
        new Trigger(() -> IO.getButtonValue(Controls.revSpeaker).get()).whileTrue(new RevSpeakerCommand());
        new Trigger(() -> IO.getButtonValue(Controls.revTape).get()).whileTrue(new RevTapeCommand());

        new Trigger(() -> IO.getButtonValue(Controls.releaseClimber).get()).toggleOnTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().releaseClimber()));
        new Trigger(() -> IO.getButtonValue(Controls.pinClimber).get()).toggleOnTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().pinClimber()));
        new Trigger(() -> IO.getButtonValue(Controls.climb).get()).whileTrue(new ClimbCommand());
        new Trigger(() -> IO.getButtonValue(Controls.invertClimb).get()).whileTrue(new ReverseClimbCommand());

        new Trigger(() -> IO.getButtonValue(Controls.toggleTBone).get()).toggleOnTrue(new TBoneCommand());
        new Trigger(() -> IO.getButtonValue(Controls.toggleDeflector).get()).onTrue(new InstantCommand(() -> DeflectorSubsystem.getInstance().toggleDeflectorState()));
    }

    @Override
    public void robotPeriodic()
    {
        readPeriodicInputs();
        CommandScheduler.getInstance().run();
        writePeriodicInputs();

    }
    public void readPeriodicInputs()
    {
        DrivetrainSubsystem.getInstance().readPeriodic();
    }

    public void writePeriodicInputs()
    {
        DrivetrainSubsystem.getInstance().writePeriodic();
    }

    @Override
    public void teleopPeriodic() {

    }
    @Override
    public void autonomousInit()
    {
        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit()
    {
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
        CommandScheduler.getInstance().schedule(new TelopController());
        
    }
}
