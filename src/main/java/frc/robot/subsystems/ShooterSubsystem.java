// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class ShooterSubsystem extends SubsystemBase implements Loggable {
  private final CANSparkMax rightMotor = new CANSparkMax(ShooterConstants.kRightMotorPort, MotorType.kBrushless);
  private final CANSparkMax leftMotor = new CANSparkMax(ShooterConstants.kLeftMotorPort, MotorType.kBrushless);
  private final VictorSPX hoodMotor = new VictorSPX(ShooterConstants.kHoodMotorPort);
  private final Servo ballServo = new Servo(ShooterConstants.kStopperServoPort);
  private final Encoder hoodEncoder = new Encoder(ShooterConstants.kHoodEncoderPortA,ShooterConstants.kHoodEncoderPortB); 
  private CANEncoder m_encoder;
// public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private CANPIDController m_pidController;
  private double m_setPoint;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();
    hoodMotor.configFactoryDefault();
    hoodMotor.setNeutralMode(NeutralMode.Brake);
    
    // m_encoder = rightMotor.getEncoder();
    leftMotor.follow(rightMotor, true);
    leftMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);
    
    m_encoder = rightMotor.getEncoder();
    
    // Initialize PID gains and settings for
    m_pidController = rightMotor.getPIDController(); // Right motor is leader
    m_pidController.setP(ShooterConstants.kP);
    m_pidController.setI(ShooterConstants.kI);
    m_pidController.setD(ShooterConstants.kD);
    m_pidController.setIZone(ShooterConstants.kIz);
    m_pidController.setFF(ShooterConstants.kFF);
     m_pidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
    leftMotor.burnFlash();
    rightMotor.burnFlash();


  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("SetPoint", m_setPoint);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
    SmartDashboard.putNumber("Hood Encoder Distance", getHoodEncoderDistance());
    SmartDashboard.putNumber("Hood Angle", getHoodAngle());
  }

  public void stop() {
    rightMotor.stopMotor();
  }

  public void start() {
    m_pidController.setReference(m_setPoint, ControlType.kVelocity);
  }

  public void idle(double idleRPM) {
    if (idleRPM < m_encoder.getVelocity()) {
      rightMotor.stopMotor();
    } else {
      m_pidController.setReference(idleRPM, ControlType.kVelocity);
    }
  }

  public void hoodUp() {
    hoodMotor.set(ControlMode.PercentOutput, -0.5);
  }

  public void hoodDown(){
    hoodMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void hoodStop(){
    hoodMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getHoodEncoderDistance(){
    return -hoodEncoder.getDistance();
  }

  public void resetEncoderDistance(){
    hoodEncoder.reset();
  }
  

  public double getHoodAngle(){
    return ShooterConstants.kHoodStartingAngle
           -(getHoodEncoderDistance()*ShooterConstants.kHoodDegreesPerCount);
  }

  
  private void servoDown(){ ballServo.set(0);}
  private void servoUp(){ ballServo.set(0.5);}
  public void toggleServo() {
    if (ballServo.get() < 0.2) {
      servoUp();
    } else {
      servoDown();
    }
  }
  
  @Config(defaultValueNumeric = 3000)
  public void setSetPoint(double setPoint) {
       m_setPoint = setPoint;
   }
}
