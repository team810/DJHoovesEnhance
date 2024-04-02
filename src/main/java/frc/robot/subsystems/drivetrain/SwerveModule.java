package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

class SwerveModule {
    private final SwerveModuleIO module;

    private final PIDController velocityController;
    private final PIDController steerController;
    private final SimpleMotorFeedforward velocityFF;
    private SwerveModuleState state;
    private SwerveModulePosition position;

    private DrivetrainSubsystem.SpeedMode mode;
    private final SwerveModuleDetails details;

    public SwerveModule(SwerveModuleDetails details)
    {
        if (Robot.isReal())
        {
            velocityController = new PIDController(.005,0,0);
            velocityFF = new SimpleMotorFeedforward(0,.00215,.00022);
            steerController = new PIDController(15,0,0);

            module = new SwerveModuleRev(details);

        } else if (Robot.isSimulation()) {

            velocityController = new PIDController(.005,0,0);
            velocityFF = new SimpleMotorFeedforward(0,.00215,.00022);
            steerController = new PIDController(15,0,0);

            module = new SwerveModuleSim(details);
        }else{
            throw new RuntimeException(
                    "The PID controls for both the drive controller " +
                    "and the steer controllers are not getting configured, " +
                    "how is the robot not real or simulated"
            );
        }

        steerController.enableContinuousInput(-Math.PI, Math.PI);
        steerController.setTolerance(.005);
        velocityController.setTolerance(0);

        position = new SwerveModulePosition();
        state = new SwerveModuleState();
        this.details = details;
        module.setState(new SwerveModuleState(0,new Rotation2d()));
    }

    public void setIdleMode(CANSparkMax.IdleMode mode)
    {
        module.setIdleMode(mode);
    }

    public void readPeriodic()
    {
        module.readPeriodic();

        position.distanceMeters = module.getWheelPosition();
        position.angle = module.getWheelAngle();
    }
    public void writePeriodic()
    {
        double targetRPM =
                (state.speedMetersPerSecond / DrivetrainConstants.DISTANCE_PER_REVOLUTION)
                        * 60 * DrivetrainConstants.DRIVE_GEAR_RATIO;
        if (!RobotState.isDisabled())
        {
            module.setState(state);

            module.setDriveVoltage(
                    velocityController.calculate(module.getWheelVelocity(), targetRPM) + velocityFF.calculate(targetRPM)
            );


            module.setSteerVoltage(
                    steerController.calculate(module.getWheelAngle().getRadians(),MathUtil.angleModulus(state.angle.getRadians()))
            );
        }
        Logger.recordOutput("Drivetrain/" + details.module.name() +
                "/TargetVelocity", targetRPM);
        Logger.recordOutput("Drivetrain/" + details.module.name() +
                "/TargetAngle", state.angle.getRadians());
        Logger.recordOutput("Drivetrain/" + details.module.name() +
                "/AtAngleSetpoint", steerController.atSetpoint());
        Logger.recordOutput("Drivetrain/" + details.module.name() +
                "/Error", steerController.getPositionError());

        module.writePeriodic();
    }

    void setState(SwerveModuleState state)
    {
        this.state = state;
    }

    void resetModulePositions()
    {
        module.resetPosition();
        position = new SwerveModulePosition();
    }


    public SwerveModulePosition getModulePosition()
    {
        if (Robot.isReal()) {
            position.distanceMeters = -position.distanceMeters;
        }
        return position;
    }

    SwerveModuleState getState()
    {
        double speedOfWheel = module.getWheelVelocity();
        speedOfWheel = (((speedOfWheel / DrivetrainConstants.DRIVE_GEAR_RATIO) / 60))
                * DrivetrainConstants.DISTANCE_PER_REVOLUTION;

        return new SwerveModuleState(speedOfWheel, module.getWheelAngle());
    }
}
