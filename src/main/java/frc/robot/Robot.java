
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.IO.IO;
import frc.robot.commands.TelopController;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot
{
    private Command autonomousCommand;

    double lastTime = 0;

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
        setUseTiming(true);
        CommandScheduler.getInstance().unregisterAllSubsystems();

        autonomousCommand = AutoBuilder.getAuto();
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
    public void teleopInit()
    {
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
        CommandScheduler.getInstance().schedule(new TelopController());
        
    }
}
