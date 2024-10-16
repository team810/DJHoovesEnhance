package frc.robot.subsystems.drivetrain;

public class DrivetrainConstants
{
    public static final int FRONT_LEFT_DRIVE_MOTOR = 3;
    public static final int FRONT_LEFT_STEER_MOTOR = 4;
    public static final int FRONT_LEFT_ENCODER = 17;

    public static final int FRONT_RIGHT_DRIVE_MOTOR = 1;
    public static final int FRONT_RIGHT_STEER_MOTOR = 2;
    public static final int FRONT_RIGHT_ENCODER = 16;

    public static final int BACK_LEFT_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_STEER_MOTOR = 6;
    public static final int BACK_LEFT_ENCODER = 18;

    public static final int BACK_RIGHT_DRIVE_MOTOR = 7;
    public static final int BACK_RIGHT_STEER_MOTOR = 8;
    public static final int BACK_RIGHT_ENCODER = 19;

    public static final int GYRO_ID = 21;

    // Neo with L2
    public static final double MAX_VELOCITY = 4.6; // MPS
    public static final double MAX_ACCELERATION = 7; // MPS
    public static final double DRIVE_GEAR_RATIO = 6.75;

    // Kraken L3+
//    public static final double MAX_VELOCITY = 5.9; // MPS
//    public static final double MAX_ACCELERATION = 7; // MPS
//    public static final double DRIVE_GEAR_RATIO = 5.36;

    public static final double NORMAL_SPEED = 4.6;
    public static final double SLOW_SPEED = 2;
    /**
     * The measurement of the front Left wheel to the front right wheel or the back left wheel to the back right wheel
     * @Unites Meters
     */
    public static final double DRIVETRAIN_TRACK_WIDTH_METERS = 0.635;
    /**
     * This is the measurement from the Front Left wheel to the back left wheel or the front right wheel to the back right wheel
     * @Unites Meters
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.635; // Width of bot in meters
    public static final double WHEEL_DIAMETER  = .1016;

    /**
     * The distance traveled for ever rotation of the wheel. PI * Wheel Diameter in inches
     */
    public static final double DISTANCE_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;

    public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / Math.hypot(DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
    public static final double MAX_ANGULAR_ACCELERATION = MAX_ACCELERATION / Math.hypot(DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final double NORMAL_ANGULAR_VELOCITY = NORMAL_SPEED / Math.hypot(DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
    public static final double SLOW_ANGULAR_VELOCITY = SLOW_SPEED / Math.hypot(DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final double MAX_ANGULAR_VELOCITY_ACCEPT_VISION_DATA = 2 * Math.PI;

    public static final String LimeLightName = "limelight-cam";

    public static final double MAX_AUTO_SPEED = 4.0;
}
