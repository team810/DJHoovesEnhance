package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import frc.robot.lib.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

public class DrivetrainSubsystem extends AdvancedSubsystem {
    private static DrivetrainSubsystem INSTANCE;

    private DrivetrainMode drivetrainMode;
    private SpeedMode speedMode;

    private final Pigeon2 gyro;

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private SwerveModulePosition frontLeftPosition;
    private SwerveModulePosition frontRightPosition;
    private SwerveModulePosition backLeftPosition;
    private SwerveModulePosition backRightPosition;

    private SwerveModuleState frontLeftTargetState;
    private SwerveModuleState frontRightTargetState;
    private SwerveModuleState backLeftTargetState;
    private SwerveModuleState backRightTargetState;

    private final SwerveDriveKinematics kinematics;

    private ChassisSpeeds targetSpeeds;
    private ChassisSpeeds telopSpeeds; // This is the chassis speeds input from joystick values in telop
    private ChassisSpeeds currentSpeeds; // This is the chassis speeds based on readings from the swerve modules

    private final SwerveDrivePoseEstimator poseEstimator;

    private DrivetrainSubsystem() {

        gyro = new Pigeon2(DrivetrainConstants.GYRO_ID);

        frontLeft = new SwerveModule(new SwerveModuleDetails(
                DrivetrainConstants.FRONT_LEFT_DRIVE_MOTOR,
                DrivetrainConstants.FRONT_LEFT_STEER_MOTOR,
                DrivetrainConstants.FRONT_LEFT_ENCODER,
                SwerveModuleEnum.frontLeft
        ));
        frontRight = new SwerveModule(new SwerveModuleDetails(
                DrivetrainConstants.FRONT_RIGHT_DRIVE_MOTOR,
                DrivetrainConstants.FRONT_RIGHT_STEER_MOTOR,
                DrivetrainConstants.FRONT_RIGHT_ENCODER,
                SwerveModuleEnum.frontRight
        ));
        backLeft = new SwerveModule(new SwerveModuleDetails(
                DrivetrainConstants.BACK_LEFT_DRIVE_MOTOR,
                DrivetrainConstants.BACK_LEFT_STEER_MOTOR,
                DrivetrainConstants.BACK_LEFT_ENCODER,
                SwerveModuleEnum.backLeft
        ));
        backRight = new SwerveModule(new SwerveModuleDetails(
                DrivetrainConstants.BACK_RIGHT_DRIVE_MOTOR,
                DrivetrainConstants.BACK_RIGHT_STEER_MOTOR,
                DrivetrainConstants.BACK_RIGHT_ENCODER,
                SwerveModuleEnum.backRight
        ));

        kinematics = new SwerveDriveKinematics(
                new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACK_WIDTH_METERS / 2.0,
                        DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Front right
                new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACK_WIDTH_METERS / 2.0,
                        -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Back left
                new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACK_WIDTH_METERS / 2.0,
                        DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Back right
                new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACK_WIDTH_METERS / 2.0,
                        -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
        );

        frontLeftTargetState = new SwerveModuleState();
        frontRightTargetState = new SwerveModuleState();
        backLeftTargetState = new SwerveModuleState();
        backRightTargetState = new SwerveModuleState();

        frontLeftPosition = frontLeft.getModulePosition();
        frontRightPosition = frontRight.getModulePosition();
        backLeftPosition = backLeft.getModulePosition();
        backRightPosition = backRight.getModulePosition();

        targetSpeeds = new ChassisSpeeds(0, 0, 0);
        telopSpeeds = new ChassisSpeeds();

        setSpeedMode(SpeedMode.normal);
        setDrivetrainMode(DrivetrainMode.telop);

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getFiledRelativeOrientationOfRobot(), getModulePositions(), new Pose2d());
    }


    @Override
    public void readPeriodic() {

        frontLeft.readPeriodic();
        frontRight.readPeriodic();
        backLeft.readPeriodic();
        backRight.readPeriodic();

        frontLeftPosition = frontLeft.getModulePosition();
        frontRightPosition = frontRight.getModulePosition();
        backLeftPosition = backLeft.getModulePosition();
        backRightPosition = backRight.getModulePosition();

        currentSpeeds = kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
    }
    @Override
    public void writePeriodic() {
        switch (drivetrainMode)
        {
            case trajectory -> {

            }
            case telop -> {
                targetSpeeds = telopSpeeds;
            }
            case off -> {
                targetSpeeds = new ChassisSpeeds(0,0,0);
            }
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromRobotRelativeSpeeds(targetSpeeds, getFiledRelativeOrientationOfRobot()));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.MAX_VELOCITY);

        states[0] = SwerveModuleState.optimize(states[0], frontLeft.getState().angle);
        states[1] = SwerveModuleState.optimize(states[1], frontRight.getState().angle);
        states[2] = SwerveModuleState.optimize(states[2], backLeft.getState().angle);
        states[3] = SwerveModuleState.optimize(states[3], backRight.getState().angle);

        frontLeftTargetState = states[0];
        frontRightTargetState = states[1];
        backLeftTargetState = states[2];
        backRightTargetState = states[3];

        frontLeft.setState(frontLeftTargetState);
        frontRight.setState(frontRightTargetState);
        backLeft.setState(backLeftTargetState);
        backRight.setState(backRightTargetState);

        frontLeft.writePeriodic();
        frontRight.writePeriodic();
        backLeft.writePeriodic();
        backRight.writePeriodic();

        Logger.recordOutput("Drivetrain/TargetModuleStates", frontLeftTargetState, frontRightTargetState, backLeftTargetState, backRightTargetState);
        Logger.recordOutput("Drivetrain/CurrentModuleStates", frontLeft.getState(), frontRight.getState(),backLeft.getState(), backRight.getState());
        Logger.recordOutput("Drivetrain/currentSpeeds", currentSpeeds);
        Logger.recordOutput("Drivetrain/SpeedMode", speedMode);
        Logger.recordOutput("Drivetrain/Mode", drivetrainMode);
    }
    public SwerveModulePosition[] getModulePositions()
    {
        return new SwerveModulePosition[]{frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition};
    }

    public void zeroYaw()
    {
        gyro.setYaw(0);
    }
    public void setYaw(double yaw)
    {
        gyro.setYaw(yaw);
    }
    public Rotation2d getFiledRelativeOrientationOfRobot()
    {
        return gyro.getRotation2d();
    }
    public void setDrivetrainMode(DrivetrainMode mode)
    {
        this.drivetrainMode = mode;
    }
    public void setSpeedMode(SpeedMode mode)
    {
        this.speedMode = mode;
    }
    public void setTelopSpeeds(ChassisSpeeds speeds) {this.telopSpeeds = speeds;}
    public SpeedMode getSpeedMode() {return speedMode;}

    public SwerveDriveKinematics getKinematics(){return kinematics;}

    public enum DrivetrainMode
    {
        trajectory,
        telop,
        off
    }
    public enum HeadingControlMode
    {
        headingVelocityControl,
        headingAngleControl
    }
    public enum SpeedMode
    {
        slow,
        normal
    }
    public static DrivetrainSubsystem getInstance() {
        if (INSTANCE == null)
        {
            INSTANCE = new DrivetrainSubsystem();
        }
        return INSTANCE;
    }
}




