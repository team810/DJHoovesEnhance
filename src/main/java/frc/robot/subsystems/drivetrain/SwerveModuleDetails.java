package frc.robot.subsystems.drivetrain;

class SwerveModuleDetails {
	public int driveID;
	public int steerID;
	public int encoderID;
	public SwerveModuleEnum module;

	public SwerveModuleDetails(int mDriveID, int mSteerID, int mEncoderID, SwerveModuleEnum mModule) {
		driveID = mDriveID;
		steerID = mSteerID;
		encoderID = mEncoderID;
		module = mModule;

	}
}
