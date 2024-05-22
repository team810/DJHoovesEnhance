package frc.robot.subsystems.drivetrain;

import com.choreo.lib.ChoreoTrajectoryState;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Robot;
import frc.robot.lib.AdvancedSubsystem;
import frc.robot.lib.LimelightHelpers;
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
    private ChassisSpeeds teleopSpeeds;
    private ChassisSpeeds currentSpeeds;
    private Twist2d currentTwist;

    private final SwerveDrivePoseEstimator poseEstimator;

    private SwerveDriveWheelPositions currentWheelPositions;
    private SwerveDriveWheelPositions previousWheelPositions;

    private final ProfiledPIDController yawController;
    private Rotation2d targetAngle;
    private YawControlMode yawControlMode;

    private final SwerveTrajectoryController swerveController;
    private ChoreoTrajectoryState trajectoryState;

    private DrivetrainSubsystem() {
        gyro = new Pigeon2(DrivetrainConstants.GYRO_ID);
        gyro.reset();

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
        teleopSpeeds = new ChassisSpeeds();

        setSpeedMode(SpeedMode.normal);
        setDrivetrainMode(DrivetrainMode.off);

        frontLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
        frontRight.setIdleMode(CANSparkBase.IdleMode.kBrake);
        backLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
        backRight.setIdleMode(CANSparkBase.IdleMode.kBrake);

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(), getModulePositions(), new Pose2d());
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));

        yawController = new ProfiledPIDController(15,0,1.5,new TrapezoidProfile.Constraints(DrivetrainConstants.MAX_ANGULAR_VELOCITY, DrivetrainConstants.MAX_ANGULAR_ACCELERATION), Robot.defaultPeriodSecs);
        yawController.enableContinuousInput(-Math.PI, Math.PI);
        yawController.setTolerance(Math.toRadians(.5));

        yawControlMode = YawControlMode.velocity;
        targetAngle = new Rotation2d();

        previousWheelPositions = new SwerveDriveWheelPositions(getModulePositions());
        currentWheelPositions = new SwerveDriveWheelPositions(getModulePositions());

        swerveController = new SwerveTrajectoryController(
                new PIDController(7,0,0),
                new PIDController(7,0,0),
                new PIDController(10,0,0)
        );
        trajectoryState = new ChoreoTrajectoryState(0,0,0,0,0,0,0);
        Shuffleboard.getTab("Drivetrain").add(yawController);
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

        currentWheelPositions = new SwerveDriveWheelPositions(getModulePositions());

        currentTwist = kinematics.toTwist2d(previousWheelPositions, currentWheelPositions);
        previousWheelPositions = currentWheelPositions;

        currentSpeeds = kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
        gyro.getSimState().addYaw(-Math.toDegrees(currentSpeeds.omegaRadiansPerSecond * Robot.defaultPeriodSecs));

        poseEstimator.update(gyro.getRotation2d(), getModulePositions());

        boolean reject = false;

        LimelightHelpers.SetRobotOrientation(DrivetrainConstants.LimeLightName, poseEstimator.getEstimatedPosition().getRotation().getDegrees(), gyro.getRate(),0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(DrivetrainConstants.LimeLightName);

        if(Math.abs(Math.toRadians(gyro.getRate())) > DrivetrainConstants.MAX_ANGULAR_VELOCITY_ACCEPT_VISION_DATA)
        {
            reject = true;
        }
        if(mt2.tagCount == 0)
        {
            reject = true;
        }
        if(!reject)
        {
            poseEstimator.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);
        }
    }

    @Override
    public void writePeriodic() {
        switch (drivetrainMode)
        {
            case trajectory ->
            {
                targetSpeeds = swerveController.calculate(getRobotPose(), trajectoryState);
            }
            case teleop -> {
                targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(teleopSpeeds,getFiledRelativeOrientationOfRobot());
                switch (yawControlMode)
                {
                    case dpad, rightStick -> {
                        double plant = -yawController.calculate(getFiledRelativeOrientationOfRobot().getRadians(), targetAngle.getRadians());
                        plant = MathUtil.clamp(plant, -DrivetrainConstants.MAX_ANGULAR_VELOCITY, DrivetrainConstants.MAX_ANGULAR_VELOCITY);
                        plant = MathUtil.applyDeadband(plant/DrivetrainConstants.MAX_ANGULAR_VELOCITY,.05) * DrivetrainConstants.MAX_ANGULAR_ACCELERATION;
                        targetSpeeds.omegaRadiansPerSecond = plant;
                    }
                    case velocity -> {
                        teleopSpeeds.omegaRadiansPerSecond = targetSpeeds.omegaRadiansPerSecond;
                    }
                }
            }
            case off -> {
                targetSpeeds = new ChassisSpeeds(0,0,0);
            }
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.NORMAL_SPEED);

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

        Logger.recordOutput("Drivetrain/ModuleStates/Target", frontLeftTargetState, frontRightTargetState, backLeftTargetState, backRightTargetState);
        Logger.recordOutput("Drivetrain/ModuleStates/Current", frontLeft.getState(), frontRight.getState(),backLeft.getState(), backRight.getState());
        Logger.recordOutput("Drivetrain/Current/Speeds", currentSpeeds);
        Logger.recordOutput("Drivetrain/Current/Twist", currentTwist);

        Logger.recordOutput("Drivetrain/Trajectory/AtReference", swerveControllerAtReference());
        Logger.recordOutput("Drivetrain/Trajectory/TargetPose", trajectoryState.getPose());

        Logger.recordOutput("Drivetrain/SpeedMode", speedMode);
        Logger.recordOutput("Drivetrain/Mode", drivetrainMode);
        Logger.recordOutput("Drivetrain/RobotPose", getRobotPose());
        Logger.recordOutput("Drivetrain/Rotation", getFiledRelativeOrientationOfRobot());

        Logger.recordOutput("Drivetrain/RotationControl/Mode", yawControlMode.toString());
        Logger.recordOutput("Drivetrain/RotationControl/TargetAngle", targetAngle);
    }

    private SwerveModulePosition[] getModulePositions() {return new SwerveModulePosition[]{frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition};}
    public Rotation2d getFiledRelativeOrientationOfRobot() {return getRobotPose().getRotation();}
    public void setFiledRelativePose(Pose2d pose) {
        frontLeftPosition = new SwerveModulePosition(0,new Rotation2d());
        frontRightPosition = new SwerveModulePosition(0,new Rotation2d());
        backLeftPosition = new SwerveModulePosition(0,new Rotation2d());
        backRightPosition = new SwerveModulePosition(0,new Rotation2d());
        poseEstimator.resetPosition(gyro.getRotation2d(), new SwerveModulePosition[]{frontRightPosition, frontRightPosition, backLeftPosition, backRightPosition}, pose);
    }
    public void setDrivetrainMode(DrivetrainMode mode)
    {
        this.drivetrainMode = mode;
    }
    public void setSpeedMode(SpeedMode mode)
    {
        this.speedMode = mode;
    }
    public Pose2d getRobotPose()
    {
        return poseEstimator.getEstimatedPosition();
    }
    public void setTeleopSpeeds(ChassisSpeeds speeds) {this.teleopSpeeds = speeds;}
    public SpeedMode getSpeedMode() {return speedMode;}
    public void setYaw(double yaw)
    {
        gyro.setYaw(yaw);
    }
    public void resetGyroButton() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        {
            poseEstimator.resetPosition(gyro.getRotation2d(),getModulePositions(), new Pose2d(getRobotPose().getX(), getRobotPose().getY(), Rotation2d.fromDegrees(180)));
        }else{
            poseEstimator.resetPosition(gyro.getRotation2d(),getModulePositions(), new Pose2d(getRobotPose().getX(), getRobotPose().getY(), Rotation2d.fromDegrees(0)));
        }
    }
    public SwerveDriveKinematics getKinematics(){return kinematics;}
    public void setTrajectoryState(ChoreoTrajectoryState trajectoryState) {
        this.trajectoryState = trajectoryState;
    }
    public boolean swerveControllerAtReference()
    {
        return this.swerveController.atReference();
    }
    public ChassisSpeeds getCurrentSpeeds() {return currentSpeeds;}
    public YawControlMode getControlMode() {
        return yawControlMode;
    }
    public void setYawControlMode(YawControlMode controlMode) {
        this.yawControlMode = controlMode;
        yawController.reset(getFiledRelativeOrientationOfRobot().getRadians(), gyro.getRate());
    }
    public Rotation2d getTargetAngle() {
        return targetAngle;
    }
    public void setTargetAngle(Rotation2d targetAngle) {
        this.targetAngle = targetAngle;
    }

    public enum DrivetrainMode
    {
        trajectory,
        teleop,
        off
    }

    public enum YawControlMode
    {
        velocity,
        dpad,
        rightStick
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