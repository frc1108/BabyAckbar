// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.pantherlib.SlewLimiter1108;

public class DriveSubsystem extends SubsystemBase {
  // Constructors for Drive subsystem motors 
  CANSparkMax leftMotor1 = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  CANSparkMax leftMotor2 = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
  CANSparkMax rightMotor1 = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
  CANSparkMax rightMotor2 = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(leftMotor1, leftMotor2);
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(rightMotor1, rightMotor2);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  private final CANEncoder m_leftEncoder;
  private final CANEncoder m_rightEncoder;

  // The gyro 
  public static final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private double slewSpeed = 4;  // in units/s
  private double slewTurn = 4;
  private final SlewLimiter1108 m_speedSlew = new SlewLimiter1108(slewSpeed);
  private final SlewLimiter1108 m_turnSlew = new SlewLimiter1108(slewTurn);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_leftEncoder = leftMotor1.getEncoder();
    m_rightEncoder = rightMotor1.getEncoder();

    // Stops drive motors
    stop();
    
    // Restores default CANSparkMax settings
    leftMotor1.restoreFactoryDefaults();
    leftMotor2.restoreFactoryDefaults();
    rightMotor1.restoreFactoryDefaults();
    rightMotor2.restoreFactoryDefaults();
    
    // Set Idle mode for CANSparkMax (brake)
    leftMotor1.setIdleMode(IdleMode.kBrake);
    leftMotor2.setIdleMode(IdleMode.kBrake);
    rightMotor1.setIdleMode(IdleMode.kBrake);
    rightMotor2.setIdleMode(IdleMode.kBrake);
    
    // Set Smart Current Limit for CAN SparkMax
    leftMotor1.setSmartCurrentLimit(40, 60);
    leftMotor2.setSmartCurrentLimit(40, 60);
    rightMotor1.setSmartCurrentLimit(40, 60);
    rightMotor2.setSmartCurrentLimit(40, 60);
  
    // Burn settings into Spark MAX flash
    leftMotor1.burnFlash();
    leftMotor2.burnFlash();
    rightMotor1.burnFlash();
    rightMotor2.burnFlash();
  
    // Set drive deadband and safety 
    m_drive.setDeadband(0.05);
    m_drive.setSafetyEnabled(true);
    }


    public void periodic(){
      SmartDashboard.putNumber("Angle",getHeading());
    }
  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(m_speedSlew.calculate(-fwd), m_turnSlew.calculate(rot));
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360)*(DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Gets the average distance of the TWO encoders.
   *
   * @return the average of the TWO encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public CANEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public CANEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Stops all the Drive subsytem motors
   */
  public void stop(){
    leftMotor1.stopMotor();
    leftMotor2.stopMotor();
    rightMotor1.stopMotor();
    rightMotor2.stopMotor();
  }  


}
