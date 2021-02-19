// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  CANSparkMax frontMotor = new CANSparkMax(IntakeConstants.kFrontIntakePort, MotorType.kBrushless);
  CANSparkMax backMotor = new CANSparkMax(IntakeConstants.kBackIntakePort, MotorType.kBrushless);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // Stops drive motors
    stop();
    
    // Restores default CANSparkMax settings
    frontMotor.restoreFactoryDefaults();
    backMotor.restoreFactoryDefaults();
    
    // Set Idle mode for CANSparkMax (brake)
    frontMotor.setIdleMode(IdleMode.kCoast);
    backMotor.setIdleMode(IdleMode.kCoast);
    
    // Set Smart Current Limit for CAN SparkMax
    frontMotor.setSmartCurrentLimit(30, 40);
    backMotor.setSmartCurrentLimit(30, 40);
  
    // Burn settings into Spark MAX flash
    frontMotor.burnFlash();
    backMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    frontMotor.stopMotor();
    backMotor.stopMotor();
  }
  public void start() {
    frontMotor.set(-0.6);
    backMotor.set(-0.6); 
  }
}
