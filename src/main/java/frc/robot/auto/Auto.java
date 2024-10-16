package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.MechanismState;
import frc.robot.subsystems.deflector.DeflectorSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.intake.IntakeStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterMode;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.HashMap;

public class Auto {

    private final HashMap<Autos, Command> autos = new HashMap<>();
    public Auto() {
        PIDConstants translationController = new PIDConstants(2, 0, 0);
        PIDConstants rotationController = new PIDConstants(2, 0, 0);

        ReplanningConfig replanningConfig = new ReplanningConfig(
                true,
                true
        );

        HolonomicPathFollowerConfig holonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
                translationController,
                rotationController,
                DrivetrainConstants.MAX_AUTO_SPEED,
                DrivetrainConstants.DRIVETRAIN_TRACK_WIDTH_METERS / 2,
                replanningConfig,
                .020
        );
        AutoBuilder.configureHolonomic(
                () -> DrivetrainSubsystem.getInstance().getRobotPose(),
                DrivetrainSubsystem.getInstance()::setFiledRelativePose,
                () -> DrivetrainSubsystem.getInstance().getCurrentSpeeds(),
                DrivetrainSubsystem.getInstance()::setTrajectorySpeeds,
                holonomicPathFollowerConfig,
                 this::shouldFlip,
                DrivetrainSubsystem.getInstance()
        );

        NamedCommands.registerCommand("ScoreSub",
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            ShooterSubsystem.getInstance().setShooterMode(ShooterMode.Subwoofer);
                            DeflectorSubsystem.getInstance().setDeflectorState(MechanismState.deployed);
                        }),
                        new WaitCommand(.5),
                        new InstantCommand(() -> IntakeSubsystem.getInstance().setState(IntakeStates.fire)),
                        new WaitCommand(.3),
                        new InstantCommand(() -> {
                            IntakeSubsystem.getInstance().setState(IntakeStates.off);
                            ShooterSubsystem.getInstance().setShooterMode(ShooterMode.off);
                            DeflectorSubsystem.getInstance().setDeflectorState(MechanismState.stored);
                        })
                )
        );
        NamedCommands.registerCommand("RevSub",
                new InstantCommand(() -> {
                    ShooterSubsystem.getInstance().setShooterMode(ShooterMode.Subwoofer);
                    DeflectorSubsystem.getInstance().setDeflectorState(MechanismState.deployed);
                }
        ));
        NamedCommands.registerCommand("RevFar",
                new InstantCommand(() -> {
                    ShooterSubsystem.getInstance().setShooterMode(ShooterMode.Tape);
                    DeflectorSubsystem.getInstance().setDeflectorState(MechanismState.stored);
                })
        );
        NamedCommands.registerCommand("RevShooterPause",
                new WaitCommand(.5)
        );
        NamedCommands.registerCommand("Fire",
                new SequentialCommandGroup(
                        new InstantCommand(() -> IntakeSubsystem.getInstance().setState(IntakeStates.fire)),
                        new WaitCommand(.3),
                        new InstantCommand(() -> {
                            IntakeSubsystem.getInstance().setState(IntakeStates.off);
                            ShooterSubsystem.getInstance().setShooterMode(ShooterMode.off);
                        })
                ));
        NamedCommands.registerCommand("IntakeLaser",
                new SequentialCommandGroup(
                        new IntakeAutoCommand(),
                        new InstantCommand(() -> IntakeSubsystem.getInstance().setState(IntakeStates.rev)),
                        new WaitCommand(.02),
                        new InstantCommand(() -> IntakeSubsystem.getInstance().setState(IntakeStates.off)),
                        new WaitCommand(5)
                )
        );
        NamedCommands.registerCommand("IntakeOn",
                new InstantCommand(() -> {IntakeSubsystem.getInstance().setState(IntakeStates.fwd);})
        );
        NamedCommands.registerCommand("IntakeStop",
                new InstantCommand(() -> IntakeSubsystem.getInstance().setState(IntakeStates.off))
        );
        NamedCommands.registerCommand("AllOff",
                new InstantCommand(() -> {
                    IntakeSubsystem.getInstance().setState(IntakeStates.off);
                    ShooterSubsystem.getInstance().setShooterMode(ShooterMode.off);
                    DeflectorSubsystem.getInstance().setDeflectorState(MechanismState.stored);
                })
        );
        reloadPaths();
    }

    public void reloadPaths()
    {
        autos.put(Autos.Middle, AutoBuilder.buildAuto("Middle"));
        autos.put(Autos.FourAmp,AutoBuilder.buildAuto("FourAmp"));
        autos.put(Autos.None, new InstantCommand(() -> System.out.println("No Auto Selected")));
        autos.put(Autos.ShotOnly,
                new SequentialCommandGroup(
                        new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterMode(ShooterMode.Subwoofer)),
                        new WaitCommand(.5),
                        new InstantCommand(() -> IntakeSubsystem.getInstance().setState(IntakeStates.fire)),
                        new WaitCommand(.3),
                        new InstantCommand(() -> {
                            IntakeSubsystem.getInstance().setState(IntakeStates.off);
                            ShooterSubsystem.getInstance().setShooterMode(ShooterMode.off);
                        }))
        );
        autos.put(Autos.MoveStraight,
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            DrivetrainSubsystem.getInstance().setTrajectorySpeeds(new ChassisSpeeds(1,0,0));
                            DrivetrainSubsystem.getInstance().setDrivetrainMode(DrivetrainSubsystem.DrivetrainMode.trajectory);
                        }),
                        new WaitCommand(1),
                        new InstantCommand(() -> {
                            DrivetrainSubsystem.getInstance().setTrajectorySpeeds(new ChassisSpeeds(0,0,0));
                            DrivetrainSubsystem.getInstance().setDrivetrainMode(DrivetrainSubsystem.DrivetrainMode.off);
                        })
                )
        );
        autos.put(Autos.ThreeNonAmp,AutoBuilder.buildAuto("ThreeNonAmpSide"));
        autos.put(Autos.Move, AutoBuilder.buildAuto("JustDrive"));

    }
    public Command getAuto(Autos auto)
    {
        return autos.get(auto);
    }

    private boolean shouldFlip()
    {
        if (DriverStation.getAlliance().isPresent())
        {
            return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        }else{
            return true;
        }
    }
}
