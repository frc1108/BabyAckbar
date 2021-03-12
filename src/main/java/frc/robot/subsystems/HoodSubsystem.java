// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class HoodSubsystem extends SubsystemBase implements Loggable {
  private final VictorSPX hoodMotor = new VictorSPX(ShooterConstants.kHoodMotorPort);
  private final Encoder hoodEncoder = new Encoder(ShooterConstants.kHoodEncoderPortA,ShooterConstants.kHoodEncoderPortB); 
  private final DigitalInput m_lowSwitch = new DigitalInput(ShooterConstants.kLowSwitchPort);
  private double m_setPoint;

  
  public HoodSubsystem() {
    hoodMotor.configFactoryDefault();
    hoodMotor.setNeutralMode(NeutralMode.Brake);

    this.setDefaultCommand(new RunCommand(() -> hoodStop(), this).withName("Stop"));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Hood Encoder Distance", getHoodEncoderDistance());
    SmartDashboard.putNumber("Hood Angle", getHoodAngle());
  }

  
  public void hoodUp() {

    hoodMotor.set(ControlMode.PercentOutput, -0.5);
  }

  public void hoodDown() {
    if (m_lowSwitch.get()) {
    hoodMotor.set(ControlMode.PercentOutput, 0.5);
    }
  }

  public void hoodStop() {
    hoodMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getHoodEncoderDistance() {
    return -hoodEncoder.getDistance();
  }

  public void resetEncoderDistance() {
    hoodEncoder.reset();
  }
  
  public double getHoodAngle() {
    return ShooterConstants.kHoodStartingAngle
           -(getHoodEncoderDistance()*ShooterConstants.kHoodDegreesPerCount);
  }
  @Log
  public boolean getLowSwitch(){
   return !m_lowSwitch.get();  // logic inverted
  }

  public void increaseSetPoint(double counts) {
    m_setPoint += counts;
  }

  public void decreaseSetPoint(double counts) {
   m_setPoint -= counts;
 }

 public void zeroHoodPosition(){
   m_setPoint = 0;
   hoodEncoder.reset();
 }
}
