// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.xrp.XRPRangefinder;

public class Sensor extends SubsystemBase {
  
  public static Sensor m_instance;
  private XRPRangefinder m_sensor = new XRPRangefinder();

  public static Sensor getInstance() {
    if (m_instance == null)
      m_instance = new Sensor();

    return m_instance;
  }

  /** Creates a new Sensor. */
  protected Sensor() {}

  public double getDistance() {
    return m_sensor.getDistanceInches();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
