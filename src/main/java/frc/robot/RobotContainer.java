// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AutoSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GPMSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The driver's controller
  PS4Controller m_driverController  = new PS4Controller(OIConstants.kDriverControllerPort);
  Joystick  m_copilot_1             = new Joystick(OIConstants.kCoPilotController1Port);
  Joystick  m_copilot_2             = new Joystick(OIConstants.kCoPilotController2Port);

  // The robot's subsystems
  public final DriveSubsystem  m_robotDrive    = new DriveSubsystem(m_driverController, m_copilot_1, m_copilot_2);
  public final GPMSubsystem    m_GPM           = new GPMSubsystem(m_driverController, m_copilot_1, m_copilot_2);
  public final AutoSubsystem   m_Auto          = new AutoSubsystem(m_robotDrive, m_GPM);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(new RunCommand(() -> m_robotDrive.drive(true, true),m_robotDrive));
    m_GPM.setDefaultCommand(new RunCommand(() -> m_GPM.teleopRun(),m_GPM));

    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // redirect to the auto subsystem
    return(m_Auto.getAutonomousCommand());
  }
}
