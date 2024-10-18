package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

class SwerveModuleSim implements SwerveModuleIO {
	private final FlywheelSim drive;
	private final FlywheelSim steer;

	private double driveVoltage;
	private double steerVoltage;

	private final SwerveModuleDetails details;

	/**This is the speed of the wheel after the gear ratio in rpm*/
	private double wheelVelocity;
	/**Drive position is the distance that the wheel has traveled*/
	private double drivePosition;
	/**This is the angle that the wheel is currently at*/
	private Rotation2d steerPosition;

	public SwerveModuleSim(SwerveModuleDetails details) {
		this.details = details;
		// .-0
		drive = new FlywheelSim(DCMotor.getNEO(1),1, .0000005);
		steer = new FlywheelSim(DCMotor.getNEO(1), 21.428571428571427, 0.004);

		driveVoltage = 0;
		steerVoltage = 0;

		drivePosition = 0;
		wheelVelocity = 0;
		steerPosition = new Rotation2d(0);
	}

	@Override
	public void readPeriodic() {
		drive.update(Robot.defaultPeriodSecs);
		steer.update(Robot.defaultPeriodSecs);
		wheelVelocity = drive.getAngularVelocityRPM();

		double currentSpeed =
				(((getWheelVelocity()/DrivetrainConstants.DRIVE_GEAR_RATIO) / 60) * Robot.defaultPeriodSecs) * DrivetrainConstants.DISTANCE_PER_REVOLUTION;

		drivePosition = currentSpeed + drivePosition;

		double steerVelocity = steer.getAngularVelocityRPM();
		steerVelocity = steerVelocity / 60; // RPM to RPS
		steerVelocity = steerVelocity * Robot.defaultPeriodSecs;

		steerPosition = steerPosition.rotateBy(Rotation2d.fromRotations(steerVelocity));
		steerPosition = Rotation2d.fromRadians(MathUtil.angleModulus(steerPosition.getRadians()));
	}

	@Override
	public void writePeriodic() {
		drive.setInputVoltage(driveVoltage);
		steer.setInputVoltage(steerVoltage);

		Logger.recordOutput("Drivetrain/" + details.module.name() +
				"/WheelVelocity", wheelVelocity);
		Logger.recordOutput("Drivetrain/" + details.module.name() +
				"/DriveVoltage", driveVoltage);
		Logger.recordOutput("Drivetrain/"+ details.module.name() +
				"/DriveAmpDraw", drive.getCurrentDrawAmps());

		Logger.recordOutput("Drivetrain/" + details.module.name() +
				"/SteerVoltage", steerVoltage);
		Logger.recordOutput("Drivetrain/" + details.module.name() +
				"/WheelAngle", getWheelAngle().getRadians());
		Logger.recordOutput("Drivetrain/"+ details.module.name() +
				"/SteerAmpDraw", steer.getCurrentDrawAmps());
		Logger.recordOutput("Drivetrain/"+ details.module.name() +
				"/VelocityWheelAngleMotor", steer.getAngularVelocityRadPerSec());

		if (RobotState.isDisabled())
		{
			drive.setInputVoltage(0);
			steer.setInputVoltage(0);
		}
	}

	@Override
	public void resetPosition() {
		drivePosition = 0;
	}

	@Override
	public void setDriveVoltage(double voltage) {
		driveVoltage = MathUtil.clamp(voltage, -12, 12);
	}
	@Override
	public void setSteerVoltage(double voltage) {
		steerVoltage = MathUtil.clamp(voltage, -12, 12);
	}

	@Override
	public Rotation2d getWheelAngle() {
		return steerPosition;
	}

	@Override
	public double getWheelPosition() {
		return drivePosition;
	}

	@Override
	public double getWheelVelocity() {
		return wheelVelocity;
	}
}