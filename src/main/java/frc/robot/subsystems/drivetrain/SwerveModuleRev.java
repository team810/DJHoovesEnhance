package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

class SwerveModuleRev implements SwerveModuleIO {
	private final CANSparkMax drive;
	private final CANSparkMax steer;

	private final RelativeEncoder drive_encoder;
	private final CANcoder canCoder;
	private final SwerveModuleDetails details;

	private double driveVoltage;
	private double steerVoltage;

	public SwerveModuleRev(SwerveModuleDetails details)
	{
		drive = new CANSparkMax(details.driveID, CANSparkBase.MotorType.kBrushless);
		steer = new CANSparkMax(details.steerID, CANSparkBase.MotorType.kBrushless);

		drive.restoreFactoryDefaults();
		steer.restoreFactoryDefaults();

		drive.clearFaults();
		steer.clearFaults();
		
		canCoder = new CANcoder(details.encoderID);
		this.details = details;

		drive.setSmartCurrentLimit(40);
		steer.setSmartCurrentLimit(20);

		drive.enableVoltageCompensation(12.0);
		steer.enableVoltageCompensation(12.0);

		drive.setIdleMode(CANSparkMax.IdleMode.kBrake);
		steer.setIdleMode(CANSparkMax.IdleMode.kBrake);

		drive_encoder = drive.getEncoder();

		CANcoderConfiguration configuration = new CANcoderConfiguration();
		configuration.MagnetSensor.SensorDirection =
				SensorDirectionValue.CounterClockwise_Positive;
		configuration.MagnetSensor.AbsoluteSensorRange =
				AbsoluteSensorRangeValue.Unsigned_0To1;

		drive_encoder.setPosition(0);

		driveVoltage = 0;
		steerVoltage = 0;
	}
	@Override
	public void readPeriodic() {

	}

	@Override
	public void writePeriodic() {
		Logger.recordOutput("Drivetrain/" + details.module.name() +
				"/WheelVelocity", getWheelVelocity());
		Logger.recordOutput("Drivetrain/" + details.module.name() +
				"/DriveVoltage", driveVoltage);
		Logger.recordOutput("Drivetrain/"+ details.module.name() +
				"/DriveAmpDraw", drive.getOutputCurrent());
		Logger.recordOutput("Drivetrain/"+ details.module.name() +
				"/DriveTemperature", drive.getMotorTemperature());

		Logger.recordOutput("Drivetrain/" + details.module.name() +
				"/SteerVoltage", steerVoltage);
		Logger.recordOutput("Drivetrain/" + details.module.name() +
				"/WheelAngle", getWheelAngle().getRadians());
		Logger.recordOutput("Drivetrain/"+ details.module.name() +
				"/SteerAmpDraw", drive.getOutputCurrent());
		Logger.recordOutput("Drivetrain/"+ details.module.name() +
				"/SteerTemperature", steer.getMotorTemperature());

		Logger.recordOutput("Drivetrain/" + details.module.name() + "/Revolutions",
				(drive_encoder.getPosition() / DrivetrainConstants.DRIVE_GEAR_RATIO) * (.102 * Math.PI));
	}


    @Override
    public void resetPosition() {
		drive_encoder.setPosition(0);
    }

    @Override
    public void setIdleMode(CANSparkMax.IdleMode mode) {
		drive.setIdleMode(mode);
    }

    @Override
	public void setDriveVoltage(double voltage) {
		driveVoltage = MathUtil.clamp(voltage, -12, 12);
		drive.setVoltage(voltage);
	}

	@Override
	public void setSteerVoltage(double voltage) {
		steerVoltage = MathUtil.clamp(voltage, -12, 12);
		steerVoltage = -steerVoltage;
		steer.setVoltage(steerVoltage);
	}

	@Override
	public Rotation2d getWheelAngle() {
		return new Rotation2d(
				MathUtil.angleModulus(canCoder.getPosition().getValue()
						* 2 * Math.PI)
		);
	}

	@Override
	public double getWheelVelocity() {
		return drive_encoder.getVelocity();
	}

	@Override
	public double getWheelPosition() {
		return (drive_encoder.getPosition() / DrivetrainConstants.DRIVE_GEAR_RATIO) * (.102 * Math.PI);
	}
}
