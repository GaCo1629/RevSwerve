package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.GPMConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A hatch mechanism actuated by a single {@link edu.wpi.first.wpilibj.DoubleSolenoid}. */
public class GPMSubsystem extends SubsystemBase {
     
    private final DoubleSolenoid m_liftSolenoid =
          new DoubleSolenoid(2,
          PneumaticsModuleType.REVPH,
          GPMConstants.kGPMSolenoidPorts[0],
          GPMConstants.kGPMSolenoidPorts[1]);

    private XboxController driver;
    private Joystick copilot_1;
    private Joystick copilot_2;
    private final CANSparkMax m_Arm = new CANSparkMax(GPMConstants.kArmCanId, MotorType.kBrushless);
    private final CANSparkMax m_Collector = new CANSparkMax(GPMConstants.kCollectorCanId, MotorType.kBrushless);
  
    public GPMSubsystem(XboxController driver, Joystick copilot_1, Joystick copilot_2) {
        this.driver = driver;
        this.copilot_1 = copilot_1;
        this.copilot_2 = copilot_2;
    }

    @Override
    public void periodic() {
    // Update the odometry in the periodic block
        SmartDashboard.putString("GPM Periodic", "YUP");
    }
    
    public void teleopRun() {
        // Check for any manual controls
        if (driver.getLeftBumper()) 
            liftGPM();
        else if (driver.getRightBumper())
            lowerGPM();

        if (driver.getYButton()) 
            runArm(0.2);
        else if (driver.getAButton())
            runArm(-0.2);
        else
            runArm(0);
      
        if (driver.getXButton()) 
            runCollector(0.2);
        else if (driver.getBButton())
            runCollector(-0.2);
        else
            runCollector(0);

        SmartDashboard.putString("GPM Periodic", "YUP");
    }
    
    /** Lift the Arm. */
    public void liftGPM() {
      m_liftSolenoid.set(kForward);
    }

    /** Lower the arm. */
    public void lowerGPM() {
        m_liftSolenoid.set(kReverse);
    }

    public void runArm(double speed) {
        m_Arm.set(speed);
    }

    public void runCollector(double speed) {
        m_Collector.set(speed);
    }
}
