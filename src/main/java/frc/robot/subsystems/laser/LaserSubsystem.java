package frc.robot.subsystems.laser;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import frc.robot.Robot;
import frc.robot.lib.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

public class LaserSubsystem extends AdvancedSubsystem {
    private static LaserSubsystem INSTANCE;

    private final LaserCan sensor;
    double distance = 0;

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
    public void readPeriodic() {
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

    @Override
    public void writePeriodic() {
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

