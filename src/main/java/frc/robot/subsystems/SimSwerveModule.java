package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SimSwerveModule implements IOSwerveModule {
	SwerveModuleState state = new SwerveModuleState();
	SwerveModulePosition pos = new SwerveModulePosition();

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

}
