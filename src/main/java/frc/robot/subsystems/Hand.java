// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hand extends SubsystemBase {
  
  private static Hand m_instance;

  public static Hand getInstance() {
    if (m_instance == null)
      m_instance = new Hand();

    return m_instance;
  }


  private XRPServo m_servo = new XRPServo(4);

  /** Creates a new Hand. */
  protected Hand() {}

  public void move(double position_mod) {
    m_servo.setPosition(m_servo.getPosition() + position_mod);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
