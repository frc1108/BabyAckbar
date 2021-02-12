// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.auto.ExampleTrajectory;
import frc.robot.commands.drive.FieldOrientedTurn;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HotDog;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private double defaultSpeed = 0.8;
  private double triggerSpeed = 0.5; 

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem(); 
  private final HotDog m_hotDog = new HotDog();
  
  private final Trajectories m_path = new Trajectories(m_robotDrive);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_robotDrive.setMaxOutput(defaultSpeed);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    m_driverController.getY(GenericHID.Hand.kLeft),
                    m_driverController.getX(GenericHID.Hand.kRight)),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Grab the hatch when the 'A' button is pressed.
    
    new JoystickButton(m_driverController, Button.kBumperRight.value)
        .whenPressed(() -> m_robotDrive.setMaxOutput(triggerSpeed))
        .whenReleased(() -> m_robotDrive.setMaxOutput(defaultSpeed));

    new JoystickButton(m_driverController, Button.kBumperLeft.value)
        .whenPressed(() -> m_intake.start())
        .whenReleased(() -> m_intake.stop());


        new JoystickButton(m_driverController, Button.kA.value)
        .whenPressed(() -> m_shooter.start())
        .whenReleased(() -> m_shooter.stop());
    
    new JoystickButton(m_driverController, Button.kB.value)
        .whenPressed(() -> m_hotDog.start())
        .whenReleased(() -> m_hotDog.stop());


    new JoystickButton(m_driverController, Button.kX.value)
        .whenPressed(new FieldOrientedTurn(180, m_robotDrive));

    new POVButton(m_driverController, 0)
        .whenPressed(() -> m_shooter.hoodUp())
        .whenReleased(() -> m_shooter.hoodStop());

    new POVButton(m_driverController, 180)
        .whenPressed(() -> m_shooter.hoodDown())
        .whenReleased(() -> m_shooter.hoodStop());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new ExampleTrajectory(m_path,m_robotDrive);
  }

  public void reset(){
    m_robotDrive.reset();
  }
}
