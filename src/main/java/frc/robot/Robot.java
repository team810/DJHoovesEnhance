
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.auto.Auto;
import frc.robot.auto.Autos;
import frc.robot.commands.*;
import frc.robot.commands.intake.IntakeFwdCommand;
import frc.robot.commands.intake.IntakeRevCommand;
import frc.robot.commands.intake.IntakeSourceCommand;
import frc.robot.commands.swerve.HeadingTeleopController;
import frc.robot.commands.swerve.TeleopController;
import frc.robot.lib.MechanismState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.deflector.DeflectorSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.DrivingMode;
import frc.robot.subsystems.intake.IntakeStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.laser.LaserState;
import frc.robot.subsystems.laser.LaserSubsystem;
import frc.robot.subsystems.shooter.ShooterMode;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.tbone.TBoneSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot
{
    private Command autonomousCommand;
    private final SendableChooser<DrivingMode> DriveMode = new SendableChooser<>();
    private final SendableChooser<Autos> AutoChooser = new SendableChooser<>();
    private Auto autos;
    private DriverStation.Alliance alliance;

    public Robot()
    {
        ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");

        DriveMode.addOption("Standard", DrivingMode.Standard);
        DriveMode.addOption("Heading Control", DrivingMode.HeadingControl);
        DriveMode.setDefaultOption("Standard", DrivingMode.Standard);
//        DriveMode.setDefaultOption("Heading Control", DrivingMode.HeadingControl);

        AutoChooser.addOption("None", Autos.None);
        AutoChooser.addOption("ShootOnly", Autos.ShotOnly);
        AutoChooser.addOption("Middle",Autos.Middle);
        AutoChooser.addOption("Four Amp Side", Autos.FourAmp);
        AutoChooser.addOption("Just Mode", Autos.Move);
        AutoChooser.addOption("ThreeNonAmp", Autos.ThreeNonAmp);

        competitionTab.addBoolean("Game Piece Detected", () -> (LaserSubsystem.getInstance().getLaserState() == LaserState.Detected));
        competitionTab.add("Drive Mode", DriveMode);
        competitionTab.add("Auto", AutoChooser);

        if (DriverStation.getAlliance().isPresent())
        {
            alliance = DriverStation.getAlliance().get();
        }else{
            alliance = DriverStation.Alliance.Red;
        }
    }

    @Override
    public void robotInit() {
        Logger.recordMetadata("ProjectName", "DJHooves Enhance");
        DriverStation.silenceJoystickConnectionWarning(true);

        if (isReal()) {
            Logger.addDataReceiver(new NT4Publisher());
            Logger.addDataReceiver(new WPILOGWriter());
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }
        Logger.registerURCL(URCL.startExternal());
        Logger.start();

        IO.Initialize(IO.PrimaryDriverProfiles.Leo, IO.SecondaryDriverProfiles.KnollJoystick);
//        IO.Initialize(IO.PrimaryDriverProfiles.Leo, IO.SecondaryDriverProfiles.KnollController);

        DrivetrainSubsystem.getInstance();
        ShooterSubsystem.getInstance();
        IntakeSubsystem.getInstance();
        ClimberSubsystem.getInstance();
        TBoneSubsystem.getInstance();
        LaserSubsystem.getInstance();

        setUseTiming(true);
        CommandScheduler.getInstance().unregisterAllSubsystems();
        CommandScheduler.getInstance().setPeriod(.015);

        new Trigger(IO.getButtonValue(Controls.reset_gyro)).toggleOnTrue(new InstantCommand(() -> DrivetrainSubsystem.getInstance().resetGyroButton()));

        new Trigger(IO.getButtonValue(Controls.intakeFWD)).whileTrue(new SequentialCommandGroup(
                new IntakeFwdCommand(),
                new InstantCommand(() -> IntakeSubsystem.getInstance().setState(IntakeStates.rev)),
                new WaitCommand(.1),
                new InstantCommand(() -> IntakeSubsystem.getInstance().setState(IntakeStates.off))
        ));

        new Trigger(IO.getButtonValue(Controls.intakeFWD)).toggleOnFalse(
          new InstantCommand(() -> IntakeSubsystem.getInstance().setState(IntakeStates.off))
        );

        new Trigger(IO.getButtonValue(Controls.intakeREVS)).whileTrue(new IntakeRevCommand());
        new Trigger(IO.getButtonValue(Controls.sourceIntake)).whileTrue(new IntakeSourceCommand());

        new Trigger(IO.getButtonValue(Controls.fire)).whileTrue(new FireCommand());

        new Trigger(IO.getButtonValue(Controls.AmpScore)).whileTrue(new AmpScoreCommand());
        new Trigger(IO.getButtonValue(Controls.revSpeaker)).whileTrue(new RevSpeakerCommand());
        new Trigger(IO.getButtonValue(Controls.revTape)).whileTrue(new RevTapeCommand());

        new Trigger(IO.getButtonValue(Controls.releaseClimber)).toggleOnTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().releaseClimber()));
        new Trigger(IO.getButtonValue(Controls.pinClimber)).toggleOnTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().pinClimber()));
        new Trigger(IO.getButtonValue(Controls.climb)).whileTrue(new ClimbCommand());
        new Trigger(IO.getButtonValue(Controls.invertClimb)).whileTrue(new ReverseClimbCommand());

        new Trigger(IO.getButtonValue(Controls.toggleTBone)).toggleOnTrue(new TBoneCommand());
        new Trigger(IO.getButtonValue(Controls.toggleDeflector)).onTrue(new InstantCommand(() -> DeflectorSubsystem.getInstance().toggleDeflectorState()));

        new Trigger(IO.getButtonValue(Controls.turningModeToggle)).toggleOnTrue(
                new InstantCommand(() ->
                {
                    if (DrivetrainSubsystem.getInstance().getControlMode() == DrivetrainSubsystem.YawControlMode.rightStick)
                    {
                        DrivetrainSubsystem.getInstance().setYawControlMode(DrivetrainSubsystem.YawControlMode.velocity);
                    }else{
                        DrivetrainSubsystem.getInstance().setYawControlMode(DrivetrainSubsystem.YawControlMode.rightStick);

                    }
                })
        );
        autos = new Auto();
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
        ClimberSubsystem.getInstance().readPeriodic();
        TBoneSubsystem.getInstance().readPeriodic();
        ShooterSubsystem.getInstance().readPeriodic();
        IntakeSubsystem.getInstance().readPeriodic();
        DeflectorSubsystem.getInstance().readPeriodic();
        LaserSubsystem.getInstance().readPeriodic();
    }

    public void writePeriodicInputs()
    {
        DrivetrainSubsystem.getInstance().writePeriodic();
        ClimberSubsystem.getInstance().writePeriodic();
        TBoneSubsystem.getInstance().writePeriodic();
        ShooterSubsystem.getInstance().writePeriodic();
        IntakeSubsystem.getInstance().writePeriodic();
        DeflectorSubsystem.getInstance().writePeriodic();
        LaserSubsystem.getInstance().writePeriodic();
    }

    @Override
    public void teleopInit()
    {
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }

        TBoneSubsystem.getInstance().setState(MechanismState.stored);
        ShooterSubsystem.getInstance().setShooterMode(ShooterMode.off);
        IntakeSubsystem.getInstance().setState(IntakeStates.off);
        DeflectorSubsystem.getInstance().setDeflectorState(MechanismState.stored);

        if (DriveMode.getSelected() == DrivingMode.HeadingControl)
        {
            CommandScheduler.getInstance().schedule(new HeadingTeleopController());
        }else{
            CommandScheduler.getInstance().schedule(new TeleopController());
        }
    }

    @Override
    public void autonomousInit()
    {
        DrivetrainSubsystem.getInstance().setDrivetrainMode(DrivetrainSubsystem.DrivetrainMode.trajectory);
        autos.getAuto(AutoChooser.getSelected()).schedule();
//        autonomousCommand.schedule();
    }

    @Override
    public void disabledPeriodic()
    {
        if (DriverStation.getAlliance().isPresent())
        {
            if (DriverStation.getAlliance().get() != alliance)
            {
                autos.reloadPaths();
            }
        }
    }

    @Override
    public void teleopPeriodic()
    {

    }

    @Override
    public void autonomousExit() {
        DrivetrainSubsystem.getInstance().setDrivetrainMode(DrivetrainSubsystem.DrivetrainMode.off);
    }

    @Override
    public void autonomousPeriodic()
    {

    }
}
