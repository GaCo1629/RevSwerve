package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.GPMConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A hatch mechanism actuated by a single {@link edu.wpi.first.wpilibj.DoubleSolenoid}. */
public class GPMSubsystem extends SubsystemBase {
     
    private final DoubleSolenoid m_liftSolenoid =
          new DoubleSolenoid(2,
          PneumaticsModuleType.REVPH,
          GPMConstants.kGPMSolenoidPorts[0],
          GPMConstants.kGPMSolenoidPorts[1]);
    

    private final CANSparkMax m_Arm = 
          new CANSparkMax(GPMConstants.kArmCanId, MotorType.kBrushless);
  
    private final CANSparkMax m_Collector = 
          new CANSparkMax(GPMConstants.kCollectorCanId, MotorType.kBrushless);
  
      /** Grabs the hatch. */
    public CommandBase liftGPMCommand() {
      // implicitly require `this`
        return this.runOnce(() -> m_liftSolenoid.set(kForward));
    }

    /** Releases the hatch. */
    public CommandBase lowerGPMCommand() {
        // implicitly require `this`
        return this.runOnce(() -> m_liftSolenoid.set(kReverse));
    }

    public void runArm(double speed) {
        m_Arm.set(speed);
    }

    public void runCollector(double speed) {
        m_Collector.set(speed);
    }
}
