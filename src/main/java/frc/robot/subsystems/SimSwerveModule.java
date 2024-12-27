package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SimSwerveModule implements IOSwerveModule {
    // Represents the state of one swerve module. Fields are:
    //       Rotation2d angle (angle of the module)
    //       double distanceMeters (Distance measured by the wheel of the module)
	SwerveModulePosition pos = new SwerveModulePosition();

    // Represents the state of one swerve module. Fields are:
    //       Rotation2d angle (angle of the module)
    //       double speedMetersPerSecond (Speed of the wheel of the module)
    SwerveModuleState state = new SwerveModuleState();

	@Override
	public SwerveModuleState getState() {
		return state;
	}

	@Override
	public void setDesiredState(SwerveModuleState desiredState) {
		this.state = desiredState;

	}

	@Override
	public void resetEncoders() {
		pos = new SwerveModulePosition(0, pos.angle);

	}

	@Override
	public SwerveModulePosition getPosition() {
		return pos;
	}

	@Override
	public void update(double dt) {
		double dist = state.speedMetersPerSecond * dt + pos.distanceMeters;
		var angle = state.angle;
		pos = new SwerveModulePosition(dist, angle);
	}

	@Override
	public SwerveModuleState getDesiredState() {
		return null;
	}

    public double getTurnEncoderPosition() {
        return state.angle.getRadians();
     };
  
}
