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
    private final   double  m_backupDistance = -0.15;  // use negative sign to be opposite of approach direction
    private final   double  m_pitchTrip      =  2.0;  // number of degrees variation before stopping

    private TrapezoidProfile m_profile;
    private double  initialPitch;                     // the robot pitch when we start levelling.
    private double  approachSign;                     // Are we going fwd or backwards when approaching the ramp
    private BalanceStates state;
  
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
      state = BalanceStates.APPROACHING;
      addRequirements(driveSystem);
    }
  
    @Override
    public void initialize() {
      state = BalanceStates.APPROACHING;
      m_timer.restart();
      initialPitch = m_driveSystem.getPitch();
      approachSign = Math.signum(initialPitch);
      m_driveSystem.move(AutoConstants.kBalanceApproachSpeedMPS * approachSign, 0, 0, false);
    }
  
    @Override
    public void execute() {
      // What state are we in?  
      switch(state) {
        case APPROACHING:
          double deltaP = initialPitch - m_driveSystem.getPitch();
          if (Math.abs(deltaP) > m_pitchTrip) {
              m_driveSystem.stop();
              m_profile = driveForward(m_backupDistance * approachSign) ;
              nextState(BalanceStates.STOPPING);
          }
          SmartDashboard.putNumber("delta Pitch", deltaP);
          break;
        
        case STOPPING:
          m_driveSystem.move(m_profile.calculate(m_timer.get()).velocity, 0, 0, false);
          if (m_timer.hasElapsed(m_profile.totalTime())) {
            nextState(BalanceStates.WAITING);  
          }
          break;

        case WAITING:
          if (m_timer.hasElapsed(2)) {
            if (Math.abs(m_driveSystem.getPitch()) < 4 ) {
              m_driveSystem.setX();
              nextState(BalanceStates.HOLDING);
            } else {
              approachSign = Math.signum(m_driveSystem.getPitch());
              m_profile = driveForward(0.5 * approachSign) ;  
              nextState(BalanceStates.CORRECTING);
            }
          }
          break;

        case CORRECTING:
        m_driveSystem.move(m_profile.calculate(m_timer.get()).velocity, 0, 0, false);
        if (m_timer.hasElapsed(m_profile.totalTime())) {
          m_driveSystem.setX();
          nextState(BalanceStates.HOLDING);  
        }
        break;

        case HOLDING:
          break;
       
      }
      SmartDashboard.putString("Balancing", state.toString()); 
    }

    private TrapezoidProfile driveForward (double distanceM) {
      return new TrapezoidProfile(
        // Limit the max acceleration and velocity
        new TrapezoidProfile.Constraints(AutoConstants.kMaxSpeedMPS * 0.5, AutoConstants.kMaxAccelerationMPS2),
        // End at desired position in meters; implicitly starts at 0
        new TrapezoidProfile.State(distanceM, 0 )); 
    }

    private void nextState(BalanceStates newState) {
      state = newState;
      m_timer.restart();
    }
  
    @Override
    public void end(boolean interrupted) {
      m_driveSystem.stop();
      m_timer.stop();
    }
  
    @Override
    public boolean isFinished() {
      return (state == BalanceStates.HOLDING);
    }
  }
  