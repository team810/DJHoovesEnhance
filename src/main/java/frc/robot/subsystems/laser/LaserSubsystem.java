package frc.robot.subsystems.laser;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class LaserSubsystem extends SubsystemBase {
    private static LaserSubsystem INSTANCE;

    private final LaserCan sensor;
    double distance = 0; // Distance Measurement form sensor in mm

    private LaserState state;

    private LaserSubsystem() {
        sensor = new LaserCan(LaserConstants.ID);

        if (Robot.isReal())
        {
            distance = sensor.getMeasurement().distance_mm;
        }else{
            distance = 0;
        }

        if (MathUtil.isNear(LaserConstants.EXPECTED_DISTANCE, distance, LaserConstants.TOLERANCE))
        {
            state = LaserState.Detected;
        }else{
            state = LaserState.Undetected;
        }
    }
    public LaserState getLaserState()
    {
        return state;
    }

    @Override
    public void periodic() {
        if (Robot.isReal())
        {
            distance = sensor.getMeasurement().distance_mm;
        }else{
            distance = 0;
        }
        if (MathUtil.isNear(LaserConstants.EXPECTED_DISTANCE, distance, LaserConstants.TOLERANCE))
        {
            state = LaserState.Detected;
        }else{
            state = LaserState.Undetected;
        }

        Logger.recordOutput("LaserSensor/Distance", distance);
        Logger.recordOutput("LaserSensor/State", state);
    }

    public static LaserSubsystem getInstance()
    {
        if (INSTANCE == null)
        {
            INSTANCE = new LaserSubsystem();
        }
        return INSTANCE;
    }
}

