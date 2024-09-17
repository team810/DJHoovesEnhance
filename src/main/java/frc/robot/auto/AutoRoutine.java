package frc.robot.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.ArrayList;

public class AutoRoutine {
    public final ArrayList<ChoreoTrajectory> trajectories;
    public final String name;
    public Command command;

    public AutoRoutine(String name) {
        this.name = name;
        trajectories = Choreo.getTrajectoryGroup(this.name);

        command = new InstantCommand(() -> System.out.println("No Command Generated"));
    }
}
