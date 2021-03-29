// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import java.util.function.DoubleSupplier;
import frc.robot.subsystems.HoodSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class ManualHood extends CommandBase {
  private final HoodSubsystem m_hood;
  private final DoubleSupplier hood_spd;
  /** Creates a new ManualHood. */
  public ManualHood(HoodSubsystem subsystem, DoubleSupplier speed) {
    m_hood = subsystem;
    hood_spd = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
