// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XRPDrivetrain;

public class BeanDrive extends Command {
  
  private XRPDrivetrain m_drive = XRPDrivetrain.getInstance();
  private DoubleSupplier m_forwardVelSup, m_backVelSup, m_rotationSup;
  
  /** Creates a new BeanDrive. */
  public BeanDrive(
    DoubleSupplier forwardVelSup, 
    DoubleSupplier backVelSup, 
    DoubleSupplier rotationSup
  ) {
    m_forwardVelSup = forwardVelSup;
    m_backVelSup = backVelSup;
    m_rotationSup = rotationSup;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Forword Velocity", m_forwardVelSup.getAsDouble());
    SmartDashboard.putNumber("Backwards Velocity", m_backVelSup.getAsDouble());
    SmartDashboard.putNumber("Rotation", m_rotationSup.getAsDouble());

    m_drive.beanDrive(
      m_forwardVelSup.getAsDouble(),
      m_backVelSup.getAsDouble(),
      m_rotationSup.getAsDouble()
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
