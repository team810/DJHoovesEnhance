package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

interface SwerveModuleIO {
	/** @param voltage -12 to 12*/
	void setDriveVoltage(double voltage);
	/** @param voltage -12 to 12*/
	void setSteerVoltage(double voltage);
	/**@return This is the current angle of the wheel -PI to PI*/
	Rotation2d getWheelAngle();
	/**@return The speed of the motor not the wheel in the units of rpm*/
	double getWheelVelocity();
	/**@return This distance traveled by the wheel in meters this value can be plugged straight into the module position class*/
	double getWheelPosition();

	void readPeriodic();
	void writePeriodic();
	/**
	 * This resets the wheel distence traveled
	 */
	 void resetPosition();

	 default void setState(SwerveModuleState state) {};
	/**
	 * Sets the ideal mode of the drive motor the default is break
	 */
	default void setIdleMode(CANSparkMax.IdleMode mode) {};
}
