// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Paths;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;


import frc.robot.pantherlib.Trajectory6391;

public class DriveSubsystem extends SubsystemBase {
  CANSparkMax leftMotor1 = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  CANSparkMax leftMotor2 = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
  CANSparkMax rightMotor1 = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
  CANSparkMax rightMotor2 = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(leftMotor1, leftMotor2);
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(rightMotor1, rightMotor2);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  private final PIDController m_leftPID = new PIDController(DriveConstants.kPDriveVel, 0, 0);
  private final PIDController m_rightPID = new PIDController(DriveConstants.kPDriveVel, 0, 0);
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts,DriveConstants.kvVoltSecondsPerMeter,DriveConstants.kaVoltSecondsSquaredPerMeter);
  
  public final double m_slewSpeed = 5;  // in units/s
  public final double m_slewTurn = 5;
  private final SlewRateLimiter m_speedSlew = new SlewRateLimiter(m_slewSpeed);
  private final SlewRateLimiter m_turnSlew = new SlewRateLimiter(m_slewTurn);

  public final int m_medianFilterWindow = 5;
  private final MedianFilter m_speedMedianFilter = new MedianFilter(m_medianFilterWindow);
  private final MedianFilter m_turnMedianFilter = new MedianFilter(m_medianFilterWindow);

  private final CANEncoder m_leftEncoder;
  private final CANEncoder m_rightEncoder;
  private final DifferentialDriveOdometry m_odometry;
  private static final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private final PhotonCamera m_camera = new PhotonCamera("SnakeEyes");


  public DriveSubsystem() {
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

    m_leftEncoder = leftMotor1.getEncoder();
    m_rightEncoder = rightMotor1.getEncoder();
    m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor);
    m_rightEncoder.setVelocityConversionFactor(DriveConstants.kEncoderVelocityConversionFactor);
    m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor);
    m_leftEncoder.setVelocityConversionFactor(DriveConstants.kEncoderVelocityConversionFactor);
  
    // Burn settings into Spark MAX flash
    leftMotor1.burnFlash();
    leftMotor2.burnFlash();
    rightMotor1.burnFlash();
    rightMotor2.burnFlash();
  
    // Set drive deadband and safety 
    m_drive.setDeadband(0.05);
    m_drive.setSafetyEnabled(true);

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    }


    public void periodic(){
      m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getPosition(), -m_rightEncoder.getPosition());
      SmartDashboard.putNumber("Angle",getHeading());
      SmartDashboard.putNumber("Left Dist", m_leftEncoder.getPosition());
      SmartDashboard.putNumber("Right Dist", -m_rightEncoder.getPosition());
    }

    public Pose2d getPose() {
      return m_odometry.getPoseMeters();
  }

  public double getVisionAngle() {
    var result = m_camera.getLatestResult();
    if (result.hasTargets()) {
      return (getHeading() - result.getBestTarget().getYaw());
    } else {
      // If we have no targets, stay still.
      return getHeading();
    }
  }

  
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    zeroHeading();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(),-m_rightEncoder.getVelocity());
} 

public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
}

public void tankDriveWithFeedforwardPID(double leftVelocitySetpoint, double rightVelocitySetpoint) {
    m_leftMotors.setVoltage(m_feedforward.calculate(leftVelocitySetpoint)
        + m_leftPID.calculate(m_leftEncoder.getVelocity(), leftVelocitySetpoint));
    m_rightMotors.setVoltage(m_feedforward.calculate(rightVelocitySetpoint)
        + m_rightPID.calculate(-m_rightEncoder.getVelocity(), rightVelocitySetpoint));
  m_drive.feed();
}

public void reset(){
  m_gyro.reset();
  m_leftEncoder.setPosition(0);
  m_rightEncoder.setPosition(0);
  m_odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(getHeading()));
}

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(m_speedSlew.calculate(-fwd), 0.8*m_turnSlew.calculate(rot));
  }

  /**
   * Drive robot with arcade controls and ramp on velocity
   * @param fwd the joystick forward 
   * @param rot the joystick rotation
   */
  public void arcadeDrivePIDFwithSlew(double fwd, double rot) {
    // Median filter on joystick input values
    fwd = m_speedMedianFilter.calculate(fwd);
    rot = m_turnMedianFilter.calculate(rot);

    // Might want to access default deadband or specify own in constants
    fwd = m_drive.applyDeadband(fwd, 0.02); //invert the joystick direction
    rot = m_drive.applyDeadband(rot, 0.02);

    // Inverse Kinematics to get wheel speeds for arcade drive (fwd inverted)
    var speeds = m_drive.arcadeDriveIK(-fwd, rot, true);

    // Slew limit the wheel speeds & apply robot max velocity value
    speeds.left = DriveConstants.kMaxSpeedMetersPerSecond*m_speedSlew.calculate(speeds.left);
    speeds.right = DriveConstants.kMaxSpeedMetersPerSecond*m_turnSlew.calculate(speeds.right);

    // PIDF for velocity control based on Encoder feedback
    m_leftMotors.setVoltage(m_feedforward.calculate(speeds.left)
        + m_leftPID.calculate(m_leftEncoder.getVelocity(), speeds.left));
    m_rightMotors.setVoltage(m_feedforward.calculate(speeds.right)
        + m_rightPID.calculate(-m_rightEncoder.getVelocity(), speeds.right));

    m_drive.feed();
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

  public double getHeadingCW() {
    // Not negating
    return Math.IEEEremainder(-m_gyro.getAngle(), 360);
  }

  public double getTurnRateCW() {
    // Not negating
    return -m_gyro.getRate();
  }

/**
   * Creates a command to follow a Trajectory on the drivetrain.
   * @param trajectory trajectory to follow
   * @return command that will run the trajectory
   */
  public Command createCommandForTrajectory(Trajectory trajectory, Boolean initPose) {
    if (initPose) {
      new InstantCommand(() -> {resetOdometry(trajectory.getInitialPose());});
    }

    resetEncoders();

    RamseteCommand ramseteCommand =  new RamseteCommand(trajectory, this::getPose,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics, this::getWheelSpeeds,
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    new PIDController(DriveConstants.kPDriveVel, 0, 0), this::tankDriveVolts, this);
    return ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0));
  }

  protected static Trajectory loadTrajectory(String trajectoryName) throws IOException {
    return TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve(Paths.get("paths", trajectoryName + ".wpilib.json")));
  }

  public Trajectory generateTrajectory(String trajectoryName, TrajectoryConfig config) {
    try {
      var filepath = Filesystem.getDeployDirectory().toPath().resolve(Paths.get("waypoints", trajectoryName));
      return Trajectory6391.fromWaypoints(filepath, config);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load auto trajectory: " + trajectoryName, false);
      return new Trajectory();
    }
  }
  public Trajectory loadTrajectoryFromFile(String filename) {
    try {
      return loadTrajectory(filename);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load auto trajectory: " + filename, false);
      return new Trajectory();
    }
  }

  public Trajectory generateTrajectoryFromFile(String filename) {
      var config = new TrajectoryConfig(1, 3);
      return generateTrajectory(filename, config);
  }
}
