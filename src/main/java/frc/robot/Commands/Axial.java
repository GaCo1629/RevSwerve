package frc.robot.Commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Axial extends CommandBase {

    private DriveSubsystem  m_driveSystem;
    private final   Timer   m_timer          = new Timer();
    private TrapezoidProfile m_profile;
    private double m_distanceM;
    private double m_speedMPS;
    
    /**
     * Creates a new TrapezoidProfileCommand that will execute the given {@link TrapezoidProfile}.
     * Output will be piped to the provided consumer function.
     *
     * @param driveSystem Access to the drive
     * @param requirements The subsystems required by this command.
     */
    public Axial(DriveSubsystem driveSystem, double distanceMeters, double speedMetersPerSecond) {
      m_driveSystem = driveSystem;
      m_distanceM = distanceMeters;
      m_speedMPS = speedMetersPerSecond;
      
      addRequirements(driveSystem);
    }
  
    @Override
    public void initialize() {
      m_profile = driveForward(m_distanceM, m_speedMPS) ;
      m_timer.restart();
    }
  
    @Override
    public void execute() {
      m_driveSystem.move(m_profile.calculate(m_timer.get()).velocity, 0, 0, false);
    }
    

    @Override
    public void end(boolean interrupted) {
      m_driveSystem.stop();
      m_timer.stop();
    }
  
    @Override
    public boolean isFinished() {
      return (m_timer.hasElapsed(m_profile.totalTime()));
    }

    private TrapezoidProfile driveForward (double distanceM, double speedMPS) {
      return new TrapezoidProfile(
        // Limit the max acceleration and velocity
        new TrapezoidProfile.Constraints(speedMPS, AutoConstants.kMaxAccelerationMPS2 / 2.0),
        // End at desired position in meters; implicitly starts at 0
        new TrapezoidProfile.State(distanceM, 0 )); 
    }
  
}
  