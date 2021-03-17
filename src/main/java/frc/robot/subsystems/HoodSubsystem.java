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
  private double m_hoodSpeed;

  public HoodSubsystem() {
    hoodMotor.configFactoryDefault();
    hoodMotor.setNeutralMode(NeutralMode.Brake);

    this.setDefaultCommand(new RunCommand(() -> hoodStop(), this).withName("Stop"));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Encoder Distance", getHoodEncoderDistance());
    SmartDashboard.putNumber("Hood Angle", getHoodAngle());
  }

  /**
   * 
   * @param speed positive speed 0-1 hood rotates upward
   */
  public void setHood(double speed) {
    if (speed < 0) {speed = getLowSwitch()?0:speed;} // stops at bottom when REV mag limit switch active
    if (speed > 0) {speed = (getHoodEncoderDistance() > 150)?0:speed;} // soft limit when encoder count high 
    hoodMotor.set(ControlMode.PercentOutput,-speed);  // invert speed so negative input is down 
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
}
