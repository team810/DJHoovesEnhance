package frc.robot;

public class RobotState {
    private RobotState INSTANCE;

    private RobotState()
    {

    }

    public RobotState getInstance() {
        if (INSTANCE == null)
        {
            INSTANCE = new RobotState();
        }
        return INSTANCE;
    }
}
