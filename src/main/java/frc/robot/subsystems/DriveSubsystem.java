// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

public class DriveSubsystem extends SubsystemBase {
  private IOSwerveModule m_frontLeft;
	private IOSwerveModule m_frontRight;
	private IOSwerveModule m_rearLeft;
	private IOSwerveModule m_rearRight;

  // The gyro sensor
	private final Pigeon2 m_gyro = new Pigeon2(Constants.DriveConstants.kGyroCanId);

  private final Pigeon2SimState imuSim = m_gyro.getSimState();
  private Pose2d pose2d = new Pose2d();
	public Pose2d simpos = new Pose2d();

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry m_odometry;

  private final Field2d field2d = new Field2d();
  private FieldObject2d frontLeftField2dModule = field2d.getObject("front left module");
  private FieldObject2d rearLeftField2dModule = field2d.getObject("rear left module");
  private FieldObject2d frontRightField2dModule = field2d.getObject("front right module");
  private FieldObject2d rearRightField2dModule = field2d.getObject("rear right module");


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    if (Robot.isSimulation()){
			m_frontLeft = new SimSwerveModule();
			m_frontRight = new SimSwerveModule();
			m_rearLeft = new SimSwerveModule();
			m_rearRight = new SimSwerveModule();
    } else {
      m_frontLeft = new MAXSwerveModule(
        DriveConstants.kFrontLeftDrivingCanId,
        DriveConstants.kFrontLeftTurningCanId,
        DriveConstants.kFrontLeftChassisAngularOffset);

      m_frontRight = new MAXSwerveModule(
        DriveConstants.kFrontRightDrivingCanId,
        DriveConstants.kFrontRightTurningCanId,
        DriveConstants.kFrontRightChassisAngularOffset);
  
      m_rearLeft = new MAXSwerveModule(
        DriveConstants.kRearLeftDrivingCanId,
        DriveConstants.kRearLeftTurningCanId,
        DriveConstants.kBackLeftChassisAngularOffset);
  
      m_rearRight = new MAXSwerveModule(
        DriveConstants.kRearRightDrivingCanId,
        DriveConstants.kRearRightTurningCanId,
        DriveConstants.kBackRightChassisAngularOffset);
    }

    m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

    SmartDashboard.putData(field2d);

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
      pose2d = getPose();


      field2d.setRobotPose(pose2d);

    frontLeftField2dModule.setPose(pose2d.transformBy(new Transform2d(
        Constants.DriveConstants.FRONT_LEFT_OFFSET,
        new Rotation2d(m_frontLeft.getTurnEncoderPosition()))));

    rearLeftField2dModule.setPose(pose2d.transformBy(new Transform2d(
        Constants.DriveConstants.REAR_LEFT_OFFSET,
        new Rotation2d(m_rearLeft.getTurnEncoderPosition()))));

    frontRightField2dModule.setPose(pose2d.transformBy(new Transform2d(
        Constants.DriveConstants.FRONT_RIGHT_OFFSET,
        new Rotation2d(m_frontRight.getTurnEncoderPosition()))));

    rearRightField2dModule.setPose(pose2d.transformBy(new Transform2d(
        Constants.DriveConstants.REAR_RIGHT_OFFSET,
        new Rotation2d(m_rearRight.getTurnEncoderPosition()))));

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    ChassisSpeeds speeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
    if (fieldRelative) speeds.toRobotRelativeSpeeds(Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()));
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  // Our way of doing it
  // public void zeroHeading() {
  //   m_gyro.reset();
  // }

	/** Zeroes the heading of the robot. */
	public void zeroHeading() {
		// if sideways became for/backward somehow, change it to 0 degree
		var tmp = DriverStation.getAlliance();
		double flipAngle = tmp.isPresent() && tmp.get() == Alliance.Blue ? 0 : 180;

		resetOdometry(new Pose2d(this.getPose().getTranslation(), Rotation2d.fromDegrees(flipAngle)));
		// m_gyro.setYaw(0);
	}


  public ChassisSpeeds getChassisSpeeds() {
		return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(),
				m_rearLeft.getState(), m_rearRight.getState());
	}

	// stop driving(for future commands and driveCommand)
	public void stopDrive() {
		this.drive(0, 0, 0, DriveConstants.kFieldRelative);
	}

    /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getAngularVelocityZWorld().getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  @Override
	public void simulationPeriodic() {
    // SwerveDriveWheelPositions really is just an array list of SwerveModulePosition
		// var prevPos = new SwerveDriveWheelPositions(new SwerveModulePosition[]{m_frontLeft.getPosition(),
    //   m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition()}).copy();

    var prevPos = new SwerveModulePosition[]{m_frontLeft.getPosition(),
      m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition()};
  
    m_frontLeft.update(Constants.ksimDtSec);
		m_frontRight.update(Constants.ksimDtSec);
		m_rearLeft.update(Constants.ksimDtSec);
		m_rearRight.update(Constants.ksimDtSec);


		// var currpos = new SwerveDriveWheelPositions(new SwerveModulePosition[]{m_frontLeft.getPosition(),
    //   m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition()}).copy();

    var currpos = new SwerveModulePosition[]{m_frontLeft.getPosition(),
  		m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition()};


		var twist = DriveConstants.kDriveKinematics.toTwist2d(prevPos, currpos);
		simpos = getPose().exp(twist);
		imuSim.addYaw(Math.toDegrees(twist.dtheta));
	}


}
