package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
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
    private ChassisSpeeds telopSpeeds; // This is the chassis speeds input from joystick values in telop
    private ChassisSpeeds currentSpeeds; // This is the chassis speeds based on readings from the swerve modules
    private Twist2d currentTwist;

    private final SwerveDrivePoseEstimator poseEstimator;
    private final HolonomicDriveController holonomicDriveController;

    private Trajectory.State targetTrajectoryState;

    private SwerveDriveWheelPositions currentWheelPositions;
    private SwerveDriveWheelPositions previousWheelPositions;

    private DrivetrainSubsystem() {

        gyro = new Pigeon2(DrivetrainConstants.GYRO_ID);
        Shuffleboard.getTab("Drivetrain").add("PigeonGyro", gyro);

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
        setDrivetrainMode(DrivetrainMode.off);

        frontLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
        frontRight.setIdleMode(CANSparkBase.IdleMode.kBrake);
        backLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
        backRight.setIdleMode(CANSparkBase.IdleMode.kBrake);

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(), getModulePositions(), new Pose2d());
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));

        holonomicDriveController = new HolonomicDriveController(
                new PIDController(1,0,0),
                new PIDController(1,0,0),
                new ProfiledPIDController(10,0,0,
                        new TrapezoidProfile.Constraints(DrivetrainConstants.MAX_ANGULAR_VELOCITY, DrivetrainConstants.MAX_ANGULAR_ACCELERATION)
                )
        );
        holonomicDriveController.setEnabled(true);
        holonomicDriveController.setTolerance(new Pose2d(.05,.05,Rotation2d.fromDegrees(1)));

        Shuffleboard.getTab("Drivetrain").add("XController", holonomicDriveController.getXController());
        Shuffleboard.getTab("Drivetrain").add("YController", holonomicDriveController.getXController());
        Shuffleboard.getTab("Drivetrain").add("ThetaController", holonomicDriveController.getThetaController());

        targetTrajectoryState = new Trajectory.State();

        previousWheelPositions = new SwerveDriveWheelPositions(getModulePositions());
        currentWheelPositions = new SwerveDriveWheelPositions(getModulePositions());
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
        gyro.getSimState().addYaw(Rotation2d.fromRadians(currentSpeeds.omegaRadiansPerSecond * Robot.defaultPeriodSecs).getDegrees());

        poseEstimator.update(gyro.getRotation2d(), getModulePositions());

        LimelightHelpers.PoseEstimate limelightMeasurement;
        if (DriverStation.getAlliance().isPresent()) {

            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            {
                limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(DrivetrainConstants.LimeLightName);
                if(limelightMeasurement.tagCount >= 2)
                {
                    poseEstimator.addVisionMeasurement(
                            limelightMeasurement.pose,
                            limelightMeasurement.timestampSeconds);
                }
            }
        }
    }
    @Override
    public void writePeriodic() {
        double invert = 1;
        switch (drivetrainMode)
        {
            case trajectory ->
            {
                targetSpeeds = holonomicDriveController.calculate(
                        getRobotPose(),
                        targetTrajectoryState,
                        targetTrajectoryState.poseMeters.getRotation()
                );

            }
            case telop -> {
                targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(telopSpeeds,getFiledRelativeOrientationOfRobot());
                if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                    invert = -1;
                }

            }
            case off -> {
                targetSpeeds = new ChassisSpeeds(0,0,0);
            }
        }

        targetSpeeds = new ChassisSpeeds(targetSpeeds.vxMetersPerSecond * invert, targetSpeeds.vyMetersPerSecond * invert, targetSpeeds.omegaRadiansPerSecond);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetSpeeds);

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

        frontLeftPosition.distanceMeters = frontLeftPosition.distanceMeters * invert;
        frontRightPosition.distanceMeters = frontRightPosition.distanceMeters * invert;
        backLeftPosition.distanceMeters = backLeftPosition.distanceMeters * invert;
        backRightPosition.distanceMeters = backRightPosition.distanceMeters * invert;

        Logger.recordOutput("Drivetrain/TargetModuleStates", frontLeftTargetState, frontRightTargetState, backLeftTargetState, backRightTargetState);
        Logger.recordOutput("Drivetrain/CurrentModuleStates", frontLeft.getState(), frontRight.getState(),backLeft.getState(), backRight.getState());
        Logger.recordOutput("Drivetrain/currentSpeeds", currentSpeeds);
        Logger.recordOutput("Drivetrain/SpeedMode", speedMode);
        Logger.recordOutput("Drivetrain/Mode", drivetrainMode);
        Logger.recordOutput("Drivetrain/RobotPose", getRobotPose());
        Logger.recordOutput("Drivetrain/Rotation", getFiledRelativeOrientationOfRobot());
        Logger.recordOutput("Drivetrain/targetTrajectory", targetTrajectoryState.poseMeters);
    }

    private SwerveModulePosition[] getModulePositions() {return new SwerveModulePosition[]{frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition};}
    public Rotation2d getFiledRelativeOrientationOfRobot() {return getRobotPose().getRotation();}
    public void setFiledRelativePose(Pose2d pose)
    {
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
    public void setTelopSpeeds(ChassisSpeeds speeds) {this.telopSpeeds = speeds;}
    public SpeedMode getSpeedMode() {return speedMode;}
    public void zeroYaw()
    {
        gyro.setYaw(0);
    }
    public void setYaw(double yaw)
    {
        gyro.setYaw(yaw);
    }
    public void setTargetTrajectoryState(Trajectory.State state)
    {
        targetTrajectoryState = state;
    }
    public boolean trajectoryControllerAtSetpoint(){return holonomicDriveController.atReference();}
    public void setTrajectoryState(Trajectory.State state) {this.targetTrajectoryState = state;}

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




