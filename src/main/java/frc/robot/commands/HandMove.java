// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hand;

public class HandMove extends Command {
  
  private Hand m_hand = Hand.getInstance(); 
  private DoubleSupplier m_position_mod;

  /** Creates a new HandMove. */
  public HandMove(DoubleSupplier position_mod) {
    m_position_mod = position_mod;

    addRequirements(m_hand);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is  scheduled.
  @Override
  public void execute() {
    double position_mod = -m_position_mod.getAsDouble();
    double deadband = 0.5;
    if (position_mod < deadband && position_mod > -deadband) return;
    m_hand.move(position_mod * 0.05);
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
