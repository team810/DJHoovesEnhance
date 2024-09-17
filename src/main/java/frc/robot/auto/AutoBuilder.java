package frc.robot.auto;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FireCommand;
import frc.robot.commands.RevSpeakerCommand;
import frc.robot.subsystems.shooter.ShooterMode;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.HashMap;


public class AutoBuilder {
    private final HashMap<Autos, AutoRoutine> autoRoutines = new HashMap<>();
    private final SendableChooser<Autos> autosChooser= new SendableChooser<>();

    private Autos autos;
    
    public AutoBuilder() {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
        autoTab.add(autosChooser);

        autosChooser.addOption("None", Autos.NONE);
        autosChooser.setDefaultOption("None", Autos.NONE);

        autos = autosChooser.getSelected();

        // Start building autos
        AutoRoutine middle = new AutoRoutine("middle");
        middle.command = new SequentialCommandGroup(
                new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterMode(ShooterMode.Subwoofer));
                new WaitCommand(.75),
                new FireCommand(),
        );
        autoRoutines.put(Autos.MIDDLE, middle);
    }

    public void periodic() // Only should be run when disabled
    {
        if (autos != autosChooser.getSelected())
        {

        }
    }

    public static Command getAuto() {
        return null;
    }
}
