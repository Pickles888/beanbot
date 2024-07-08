// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class XRPDrivetrain extends SubsystemBase {

  private static XRPDrivetrain m_instance;

  public static XRPDrivetrain getInstance() {
    if (m_instance == null)
      m_instance = new XRPDrivetrain();

    return m_instance;
  }

  private static final double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double kCountsPerMotorShaftRev = 12.0;
  private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
  private static final double kWheelDiameterInch = 2.3622; // 60 mm

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Gyro
  private final XRPGyro m_gyro = new XRPGyro();

  // Odometry
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
    Rotation2d.fromDegrees(m_gyro.getAngle()), 
    m_leftEncoder.getDistance(), m_rightEncoder.getDistance(),
    getPose()
  );

  /** Creates a new XRPDrivetrain. */
  protected XRPDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
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
    return new Pose2d(
      m_gyro.getRateX(),
      m_gyro.getRateY(),
      Rotation2d.fromDegrees(m_gyro.getAngle())
    );
  }

  public void resetPose() {
    m_gyro.reset();
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
