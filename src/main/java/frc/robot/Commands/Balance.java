package frc.robot.Commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Balance  extends CommandBase {

    private DriveSubsystem  m_driveSystem;
    private final   Timer   m_timer          = new Timer();
    private final   double  m_backupDistance = -0.1;  // use negative sign to be opposite of approach direction
    private final   double  m_pitchTrip      =  2.0;  // number of degrees variation before stopping

    private TrapezoidProfile m_profile;
    private double  initialPitch;                     // the robot pitch when we start levelling.
    private double  approachSign;                     // Are we going fwd or backwards when approaching the ramp
    private boolean levelling;                        // Have we stopped approaching and moved into leveling process (reversing direction).
  
    /**
     * Creates a new TrapezoidProfileCommand that will execute the given {@link TrapezoidProfile}.
     * Output will be piped to the provided consumer function.
     *
     * @param driveSystem Access to the drive
     * @param requirements The subsystems required by this command.
     */
    public Balance(DriveSubsystem driveSystem) {
      m_driveSystem = driveSystem;
      m_profile = null;
      initialPitch = 0;
      approachSign = 1.0;
      levelling = false;
      addRequirements(driveSystem);
    }
  
    @Override
    public void initialize() {
      initialPitch = m_driveSystem.getPitch();
      approachSign = Math.signum(initialPitch);
      m_driveSystem.move(AutoConstants.kBalanceApproachSpeedMPS * approachSign, 0, 0, false);
      m_timer.restart();
    }
  
    @Override
    public void execute() {
      // What state are we in?  
      if (levelling) {
        // look for pitch to start changing 
        if (Math.abs(initialPitch - m_driveSystem.getPitch()) > m_pitchTrip) {
            m_driveSystem.stop();
            m_profile = new TrapezoidProfile(
                // Limit the max acceleration and velocity
                new TrapezoidProfile.Constraints(AutoConstants.kMaxSpeedMPS * 0.5, AutoConstants.kMaxAccelerationMPS2),
                // End at desired position in meters; implicitly starts at 0
                new TrapezoidProfile.State(m_backupDistance * approachSign, 0) );
                m_timer.restart();
        }
      } else {
        m_driveSystem.move(m_profile.calculate(m_timer.get()).velocity, 0, 0, false);
      }
    }
  
    @Override
    public void end(boolean interrupted) {
      m_driveSystem.stop();
      m_timer.stop();
    }
  
    @Override
    public boolean isFinished() {
      if (levelling) {
        return (m_timer.hasElapsed(m_profile.totalTime()));
      } else {
        return (m_timer.hasElapsed(5.0));  // give up after 5 sec
      }
    }
  }
  