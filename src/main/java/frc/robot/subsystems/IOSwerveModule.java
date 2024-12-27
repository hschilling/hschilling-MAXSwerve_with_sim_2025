package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface IOSwerveModule {
	public SwerveModuleState getState();
	public SwerveModulePosition getPosition();
	public void setDesiredState(SwerveModuleState desiredState);
	public void resetEncoders();
	public void update(double dt); // TODO: Sim hack
	public SwerveModuleState getDesiredState();
}
