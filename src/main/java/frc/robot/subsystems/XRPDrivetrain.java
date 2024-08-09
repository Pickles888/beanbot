// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class XRPDrivetrain extends SubsystemBase {

  private static XRPDrivetrain m_instance;

  public static XRPDrivetrain getInstance() {
    if (m_instance == null)
      m_instance = new XRPDrivetrain();

    return m_instance;
  }

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(Constants.IDs.kLeftMotor);
  private final XRPMotor m_rightMotor = new XRPMotor(Constants.IDs.kRightMotor);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(
    Constants.IDs.kLeftEncoderA, 
    Constants.IDs.kLeftEncoderB
  );
  private final Encoder m_rightEncoder = new Encoder(
    Constants.IDs.kRightEncoderA, 
    Constants.IDs.kRightEncoderB
  );

  // Encoder velocities
  private double m_leftVel = 0;
  private double m_rightVel = 0;

  // Current distances
  private double m_leftCurrentDist = 0;
  private double m_rightCurrentDist = 0; 

  // Previous distances
  private double m_leftPastDist = 0;
  private double m_rightPastDist = 0;

  // time stuff (not hacky at all totaly please just roll with it)
  private double m_currentTime = 0;
  private double m_pastTime = 0;

  // Gyro
  private final XRPGyro m_gyro = new XRPGyro();

  // Odometry
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
    getRotation(),
    m_leftEncoder.getDistance(), m_rightEncoder.getDistance()
  );

  // Kinematics
  DifferentialDriveKinematics m_kinematics =
    new DifferentialDriveKinematics(0.155);

  /** Creates a new XRPDrivetrain. */
  protected XRPDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse(
      (Math.PI * DriveConstants.kWheelDiameterMeter) / DriveConstants.kCountsPerRevolution
    );
    m_rightEncoder.setDistancePerPulse(
      (Math.PI * DriveConstants.kWheelDiameterMeter) / DriveConstants.kCountsPerRevolution
    );
    resetEncoders();

    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);

    // Configure AutoBuilder last
    AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::gotCurrentSpeeds, // Current ChassisSpeeds supplier
            this::drive, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              Optional<Alliance> alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  public void wallmartFieldOrientedDrive(double vel, double angle) {
    double angleDiff = m_gyro.getAngle() - angle;

    if (angleDiff <= DriveConstants.fieldOrientedLeeway && angleDiff >= -DriveConstants.fieldOrientedLeeway) {
      drive(vel, vel);
      return;
    }
    
    if (angleDiff < -DriveConstants.fieldOrientedLeeway) {
      double rotateVel = Math.max(angleDiff / DriveConstants.fieldOrientedRotateSpeed, -1);
      
      rotate(rotateVel);
    } else if (angleDiff > DriveConstants.fieldOrientedLeeway) {
      double rotateVel = Math.min(angleDiff / DriveConstants.fieldOrientedRotateSpeed, 1);
      
      rotate(rotateVel);
    }
  }

  private void rotate(double vel) {
    drive(vel, -vel);
  }

  public void beanDrive(double vel, double rotationVel) {
    drive(vel + rotationVel, vel - rotationVel);
  }

  public void beanDrive(double forwardVel, double backVel, double rotationVel) {
    double vel = forwardVel - backVel;

    drive(vel + rotationVel, vel - rotationVel);
  }

  public void drive(double leftVel, double rightVel) {
    m_leftMotor.set(leftVel);
    m_rightMotor.set(rightVel);
  }

  public void drive(ChassisSpeeds chassisSpeed) {
    beanDrive(
      chassisSpeed.vxMetersPerSecond,
      chassisSpeed.omegaRadiansPerSecond
    );  
  }

  public Pose2d getPose() {
    return m_odometry.update(
      getRotation(),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance()
    );
  }

  private Rotation2d getRotation() {
    return Rotation2d.fromDegrees(m_gyro.getAngle());
  }

  public void resetPose(Pose2d pose) {
    m_gyro.reset();
    resetEncoders();
    m_odometry.resetPosition(
	getRotation(),
	m_leftEncoder.getDistance(),
	m_rightEncoder.getDistance(),
	pose
    );
  }

  public ChassisSpeeds gotCurrentSpeeds() {
    return m_kinematics.toChassisSpeeds(
	new DifferentialDriveWheelSpeeds(
	    m_leftVel,
	    m_rightVel
	)
    );
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceMeter() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeter() {
    return m_rightEncoder.getDistance();
  }

  @Override
  public void periodic() {
    m_leftCurrentDist = getLeftDistanceMeter();
    m_rightCurrentDist = getRightDistanceMeter();

    m_currentTime = Timer.getFPGATimestamp();

    double dt = m_currentTime - m_pastTime;

    // Change of distance / change in time
    m_leftVel = (m_leftCurrentDist - m_leftPastDist) / dt;
    m_rightVel = (m_rightCurrentDist - m_rightPastDist) / dt;

    // Cap time (velocity) (nate is mean) (no cap)
    m_leftVel = Math.min(DriveConstants.kMaxVel, m_leftVel); 
    m_rightVel = Math.min(DriveConstants.kMaxVel, m_rightVel); 

    m_pastTime = m_currentTime;

    m_leftPastDist = m_leftCurrentDist;
    m_rightPastDist = m_rightCurrentDist;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
