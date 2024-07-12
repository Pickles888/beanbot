// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
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
  public final XRPMotor m_leftMotor = new XRPMotor(Constants.IDs.kLeftMotor);
  public final XRPMotor m_rightMotor = new XRPMotor(Constants.IDs.kRightMotor);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  public final Encoder m_leftEncoder = new Encoder(
    Constants.IDs.kLeftEncoderA, 
    Constants.IDs.kLeftEncoderB
  );
  public final Encoder m_rightEncoder = new Encoder(
    Constants.IDs.kRightEncoderA, 
    Constants.IDs.kRightEncoderB
  );

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
      (Math.PI * DriveConstants.kWheelDiameterInch) / DriveConstants.kCountsPerRevolution
    );
    m_rightEncoder.setDistancePerPulse(
      (Math.PI * DriveConstants.kWheelDiameterInch) / DriveConstants.kCountsPerRevolution
    );
    resetEncoders();

    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);
  }

  public void beanDrive(double vel, double rotationVel) {
    m_leftMotor.set(vel + rotationVel);
    m_rightMotor.set(vel - rotationVel);
  }

  public void beanDrive(double forwardVel, double backVel, double rotationVel) {
    double vel = forwardVel - backVel;
    m_leftMotor.set(vel + rotationVel);
    m_rightMotor.set(vel - rotationVel);
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
    m_odometry.resetPosition(
      getRotation(), 
      new DifferentialDriveWheelPositions(
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance()
      ), 
      pose
    );
  }

  public ChassisSpeeds getCurrentSpeeds(double leftSpeeds, double rightSpeeds) {
    return m_kinematics.toChassisSpeeds(
      new DifferentialDriveWheelSpeeds(leftSpeeds, rightSpeeds)
    );
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_leftMotor.set(0.5);
    // System.out.println("beandriving:::::::");
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
