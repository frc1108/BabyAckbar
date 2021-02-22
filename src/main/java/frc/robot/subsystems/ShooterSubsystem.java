// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class ShooterSubsystem extends SubsystemBase implements Loggable {
  private final CANSparkMax m_rightMotor = new CANSparkMax(ShooterConstants.kRightMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_leftMotor = new CANSparkMax(ShooterConstants.kLeftMotorPort, MotorType.kBrushless);
  private final VictorSPX m_hoodMotor = new VictorSPX(ShooterConstants.kHoodMotorPort);
  private final Encoder m_hoodEncoder = new Encoder(ShooterConstants.kHoodEncoderPortA,ShooterConstants.kHoodEncoderPortB); 
  private CANEncoder m_encoder;
// public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private CANPIDController m_pidController;
  private double m_setPoint;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();
    
    m_leftMotor.follow(m_rightMotor, true);
    m_leftMotor.setIdleMode(IdleMode.kCoast);
    m_rightMotor.setIdleMode(IdleMode.kCoast);

    m_rightMotor.setSmartCurrentLimit(40);
    
    m_encoder = m_rightMotor.getEncoder();
    
    // Initialize PID gains and settings for
    m_pidController = m_rightMotor.getPIDController(); // Right motor is leader
    m_pidController.setP(ShooterConstants.kP);
    m_pidController.setI(ShooterConstants.kI);
    m_pidController.setD(ShooterConstants.kD);
    m_pidController.setIZone(ShooterConstants.kIz);
    m_pidController.setFF(ShooterConstants.kFF);

    // Store settings in controller flash
    m_leftMotor.burnFlash();
    m_rightMotor.burnFlash();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("SetPoint", m_setPoint);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
    SmartDashboard.putNumber("Hood Encoder Distance", getHoodEncoderDistance());
  }

  // *********** Flywheel methods ************
  /**
   * Stop the flywheel
   */
  public void stop() {
    m_rightMotor.stopMotor();
  }

  public void start() {
    m_pidController.setReference(m_setPoint, ControlType.kVelocity);
  }

  public void idle(double idleRPM) {
    if (idleRPM < m_encoder.getVelocity()) {
      m_rightMotor.set(0);
    } else {
      m_pidController.setReference(idleRPM, ControlType.kVelocity);
    }
  }

  @Config(defaultValueNumeric = 3000)
  public void setSetPoint(double setPoint) {
    // if camera has target & distance then calculate speed, else default to 2500
    // piecewise polynomial
       m_setPoint = setPoint;
   }


  // *********** Hood control methods ************
  public void hoodUp() {
    m_hoodMotor.set(ControlMode.PercentOutput, -0.5);
  }

  public void hoodDown(){
    m_hoodMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void hoodStop(){
    m_hoodMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getHoodEncoderDistance(){
    return m_hoodEncoder.getDistance();
  }
}
