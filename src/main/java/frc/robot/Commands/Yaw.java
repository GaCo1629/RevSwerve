package frc.robot.Commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Yaw extends CommandBase {

    private DriveSubsystem  m_driveSystem;
    private final   Timer   m_timer          = new Timer();
    private double m_headingR;
    private ProfiledPIDController m_yawController;
    
    /***
     * 
     * @param driveSystem
     * @param headingRadians
     * @param speedRadiansPerSecond
     */
    public Yaw(DriveSubsystem driveSystem, double headingRadians) {
      m_driveSystem = driveSystem;
      m_headingR = headingRadians;
      addRequirements(driveSystem);
    }
  
    @Override
    public void initialize() {
      m_yawController = new ProfiledPIDController(AutoConstants.kPYawController, 0, 0, AutoConstants.kYawControllerConstraints);
      m_yawController.enableContinuousInput(-Math.PI, Math.PI);
      m_yawController.reset(m_driveSystem.getHeading());
      m_yawController.setTolerance(Math.toRadians(5));
      m_timer.restart();
    }
  
    @Override
    public void execute() {
      double turnRate = m_yawController.calculate(m_driveSystem.getHeading(), m_headingR);
      m_driveSystem.move(0, 0, turnRate, false);
    }
    

    @Override
    public void end(boolean interrupted) {
      m_driveSystem.stop();
      m_timer.stop();
    }
  
    @Override
    public boolean isFinished() {
      return (m_yawController.atGoal());
    }

     
}
  