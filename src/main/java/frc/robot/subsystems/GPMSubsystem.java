package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.GPMConstants;
import frc.robot.Constants.ModuleConstants;
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
    private CANSparkMax m_armMax = new CANSparkMax(GPMConstants.kArmCanId, MotorType.kBrushless);
    private CANSparkMax m_collectorMax = new CANSparkMax(GPMConstants.kCollectorCanId, MotorType.kBrushless);
    private final AbsoluteEncoder m_armEncoder;
    private final SparkMaxPIDController m_armPIDController;

    private double m_armSetpoint = 60;

    
    //---------------------------------------

    public GPMSubsystem(XboxController driver, Joystick copilot_1, Joystick copilot_2) {
        this.driver = driver;
        this.copilot_1 = copilot_1;
        this.copilot_2 = copilot_2;

        // Factory reset, so we get the SPARKS MAX to a known state before configuring it
        // This is useful in case a SPARK MAX is swapped out.

        m_armMax.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        m_armEncoder = m_armMax.getAbsoluteEncoder(Type.kDutyCycle);
        m_armPIDController = m_armMax.getPIDController();
        m_armPIDController.setFeedbackDevice(m_armEncoder);

        
        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        m_armEncoder.setPositionConversionFactor(ModuleConstants.kArmEncoderPositionFactor);
        m_armEncoder.setVelocityConversionFactor(ModuleConstants.kArmEncoderVelocityFactor);

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        m_armEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        m_armPIDController.setP(ModuleConstants.kArmP);
        m_armPIDController.setI(ModuleConstants.kArmI);
        m_armPIDController.setD(ModuleConstants.kArmD);
        m_armPIDController.setFF(ModuleConstants.kArmFF);
        m_armPIDController.setOutputRange(ModuleConstants.kArmMinOutput, ModuleConstants.kArmMaxOutput);
        m_armPIDController.setIZone(10);
        m_armPIDController.setIMaxAccum(ModuleConstants.kArmMaxOutput, 0);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_armMax.burnFlash();

        
    }

    @Override
    public void periodic() {
    // Update the odometry in the periodic block
        
    }

    public void init() {
        m_armSetpoint = m_armEncoder.getPosition();
    }
    
    public void teleopRun() {
        // Check for any manual controls
        if (driver.getLeftBumper()) 
            liftGPM();
        else if (driver.getRightBumper())
            lowerGPM();

        if (driver.getYButtonPressed() && (m_armSetpoint < 190)) 
            runArm(m_armSetpoint += 5);
        else if (driver.getAButtonPressed() && (m_armSetpoint > 60))
            runArm(m_armSetpoint -= 5);

        if (copilot_1.getRawButtonPressed(1)) 
            runArm(m_armSetpoint = 51);
        else if (copilot_1.getRawButtonPressed(2))
            runArm(m_armSetpoint = 190);
        
    
        if (driver.getXButton()) 
            runCollector(0.2);
        else if (driver.getBButton())
            runCollector(-0.35);
        else
            runCollector(0);

        m_armPIDController.setReference(m_armSetpoint, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("Arm Motor", m_armMax.getAppliedOutput());
        SmartDashboard.putNumber("Arm Angle (deg)", m_armEncoder.getPosition());
        SmartDashboard.putNumber("Arm Setpoint (deg)", m_armSetpoint);
        SmartDashboard.putNumber("I Accum", m_armPIDController.getIAccum());
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
        m_armMax.set(speed);
    }

    public void runCollector(double speed) {
        m_collectorMax.set(speed);
    }
}
