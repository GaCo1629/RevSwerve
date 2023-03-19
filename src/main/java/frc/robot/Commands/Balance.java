package frc.robot.Commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BalanceStates;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Balance  extends CommandBase {

    private DriveSubsystem  m_driveSystem;
    private final   Timer   m_timer          = new Timer();
    private final   double  m_dangerPitch    =  25.0;  // Going uphill
    private final   double  m_balancedPitch  =   4.0;  // number of degrees variation before stopping
    private final   double  m_mountingPitch  =   4.0;  // number of degrees variation before stopping
    private final   double  m_climbingPitch  =  11.0;  // number of degrees variation before stopping
    
    private final   double  APPROACH_SPEED   =  1.2;
    private final   double  MOUNT_SPEED      =  1.0;
    private final   double  CLIMB_SPEED      =  0.5;
    private final   double  CRAWL_SPEED      =  0.2;
    private final   double  CORRECTING_SPEED =  0.1;

    private BalanceStates m_state;
    private double  m_tiltoverPitch  =  0.0;   // number of degrees variation before stopping
    private double  m_approachSign;             // Are we going fwd or backwards when approaching the ramp
     
    /**
     * Creates a new TrapezoidProfileCommand that will execute the given {@link TrapezoidProfile}.
     * Output will be piped to the provided consumer function.
     *
     * @param driveSystem Access to the drive
     * @param requirements The subsystems required by this command.
     */
    public Balance(DriveSubsystem driveSystem, boolean forwardApproach) {
      SmartDashboard.putNumber("tiltover pitch", 0);
      m_driveSystem = driveSystem;
      m_approachSign = forwardApproach ? 1.0 : -1.0;
      m_state = BalanceStates.APPROACHING;
      addRequirements(driveSystem);
    }
  
    @Override
    public void initialize() {
      double pitchAbs = Math.abs(m_driveSystem.getPitch());

      // determine if we are already on the ramp, and how far
      if (pitchAbs > m_mountingPitch) {
        m_approachSign = Math.signum(m_driveSystem.getPitch());
        m_driveSystem.move(MOUNT_SPEED * m_approachSign, 0, 0, false);
        nextState(BalanceStates.MOUNTING);  
      } else {
        m_driveSystem.move(APPROACH_SPEED * m_approachSign, 0, 0, false);
        nextState(BalanceStates.APPROACHING);
      }
    }
  
    @Override
    public void execute() {

      double pitchAbs = Math.abs(m_driveSystem.getPitch());
      // What state are we in?  
      switch(m_state) {

        case APPROACHING:
          if (pitchAbs > m_mountingPitch) {
            m_driveSystem.move(MOUNT_SPEED * m_approachSign, 0, 0, false);
            nextState(BalanceStates.MOUNTING);  
          } else if (m_timer.hasElapsed(3.5)) {
            // we have messed up, just stop.
            m_driveSystem.stop();
            nextState(BalanceStates.HOLDING);
          }
          break;

        case MOUNTING:
          if (pitchAbs > m_climbingPitch) {
            m_driveSystem.move(CLIMB_SPEED * m_approachSign, 0, 0, false);
            nextState(BalanceStates.CLIMBING);  
          } else if (m_timer.hasElapsed(3.5)) {
            // we have messed up, just stop.
            m_driveSystem.stop();
            nextState(BalanceStates.HOLDING);
          }
          break;

        case CLIMBING:
          if (pitchAbs > m_dangerPitch) {
            m_driveSystem.stop();
          } else if (m_timer.hasElapsed(2.0)) {
            m_tiltoverPitch = Math.max(pitchAbs - 2.0, 10.0) ;
            m_driveSystem.move(CRAWL_SPEED * m_approachSign, 0, 0, false);
            nextState(BalanceStates.CRAWLING); 
            SmartDashboard.putNumber("tiltover pitch", m_driveSystem.getPitch());
          } else {
            m_driveSystem.move(CLIMB_SPEED * m_approachSign, 0, 0, false);
          }
          break;
        
        case CRAWLING:
          if (pitchAbs < m_tiltoverPitch) {
            m_driveSystem.stop();
            m_tiltoverPitch = pitchAbs;
            SmartDashboard.putNumber("tiltover pitch", m_driveSystem.getPitch());
            nextState(BalanceStates.WAITING); 
            
          } else {
            m_driveSystem.move(CRAWL_SPEED * m_approachSign, 0, 0, false);
          }
          break;
        
        case WAITING:
          if (m_timer.hasElapsed(0.50)) {
            if (Math.abs(m_driveSystem.getPitch()) < m_balancedPitch ) {
              m_driveSystem.setX();
              nextState(BalanceStates.HOLDING);
            } else {
              nextState(BalanceStates.SLOW_CORRECTING);
            }
          }
          break;

          case SLOW_CORRECTING:
          m_driveSystem.move(CORRECTING_SPEED * Math.signum(m_driveSystem.getPitch()) , 0, 0, false);
          if (Math.abs(m_driveSystem.getPitch()) < m_tiltoverPitch ) {
            m_driveSystem.setX();
            nextState(BalanceStates.HOLDING);  
          }
          break;

        case HOLDING:
          m_driveSystem.setX();
          break;
       
      }
      SmartDashboard.putString("Balancing", m_state.toString()); 
    }
    

    private TrapezoidProfile driveForward (double distanceM) {
      return new TrapezoidProfile(
        // Limit the max acceleration and velocity
        new TrapezoidProfile.Constraints(CLIMB_SPEED, AutoConstants.kMaxAccelerationMPS2 / 2.0),
        // End at desired position in meters; implicitly starts at 0
        new TrapezoidProfile.State(distanceM, 0 )); 
    }

    private void nextState(BalanceStates newState) {
      m_state = newState;
      m_timer.restart();
    }
  
    @Override
    public void end(boolean interrupted) {
      m_driveSystem.stop();
      m_timer.stop();
    }
  
    @Override
    public boolean isFinished() {
      return (m_state == BalanceStates.HOLDING);
    }
  }
  