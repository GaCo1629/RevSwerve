package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import java.util.zip.ZipOutputStream;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Positions;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.GPMConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A hatch mechanism actuated by a single {@link edu.wpi.first.wpilibj.DoubleSolenoid}. */
public class GPMSubsystem extends SubsystemBase {
     
    private final DoubleSolenoid m_liftSolenoid =
          new DoubleSolenoid(2,
          PneumaticsModuleType.REVPH,
          GPMConstants.kGPMSolenoidPorts[0],
          GPMConstants.kGPMSolenoidPorts[1]);

    private PS4Controller driver;
    private Joystick copilot_1;
    private Joystick copilot_2;
    private CANSparkMax m_armMax = new CANSparkMax(GPMConstants.kArmCanId, MotorType.kBrushless);
    private CANSparkMax m_collectorMax = new CANSparkMax(GPMConstants.kCollectorCanId, MotorType.kBrushless);
    private final AbsoluteEncoder m_armEncoder;
    //private final SparkMaxPIDController m_armPIDController;

    private double m_armSetpoint ;
    private ProfiledPIDController m_armPID ;
       
    public GPMSubsystem(PS4Controller driver, Joystick copilot_1, Joystick copilot_2) {
        this.driver = driver;
        this.copilot_1 = copilot_1;
        this.copilot_2 = copilot_2;

        m_armPID = new ProfiledPIDController(GPMConstants.kArmP, GPMConstants.kArmI, GPMConstants.kArmD,
                                            new Constraints(GPMConstants.kMaxVelocity, GPMConstants.kMaxAcceleration));
        m_armPID.setIntegratorRange(-GPMConstants.kArmMaxI, GPMConstants.kArmMaxI);
       
        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        m_armEncoder = m_armMax.getAbsoluteEncoder(Type.kDutyCycle);
        
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block

    }

    public void init() {
        newArmSetpoint(m_armEncoder.getPosition());
        m_armPID.reset(m_armEncoder.getPosition());
    
    }
    
    public void teleopRun() {
        // Check for any manual controls
        if (driver.getL1ButtonPressed()) 
            liftGPM();
        else if (driver.getR1ButtonPressed())
            lowerGPM();

        if (driver.getTriangleButtonPressed() && (m_armSetpoint < 0.6)) 
            newArmSetpoint(m_armSetpoint + 0.02);
        else if (driver.getCrossButtonPressed() && (m_armSetpoint > .14))
            newArmSetpoint(m_armSetpoint - 0.02);

        if (copilot_1.getRawButtonPressed(OIConstants.kCP1Retract)) 
            newArmSetpoint(AutoConstants.kArmHome);
        else if (copilot_1.getRawButtonPressed(OIConstants.kCP1InCone)) 
            newArmSetpoint(AutoConstants.kArmHumCone);
        else if (copilot_1.getRawButtonPressed(OIConstants.kCP1InCube)) 
            newArmSetpoint(AutoConstants.kArmHumCube);
        else if (copilot_1.getRawButtonPressed(OIConstants.kCP1InGround)) 
            newArmSetpoint(AutoConstants.kArmGround);
        else if (copilot_1.getRawButtonPressed(OIConstants.kCP1OutCone)){ 
            if(Positions.gridLvl == 1){
                newArmSetpoint(AutoConstants.kArmConeBot);
            } else if(Positions.gridLvl == 2){
                newArmSetpoint(AutoConstants.kArmConeMid);
            } else if(Positions.gridLvl == 3){
                newArmSetpoint(AutoConstants.kArmConeTop);
            }
        }
        else if (copilot_1.getRawButtonPressed(OIConstants.kCP1OutCube)){
            if(Positions.gridLvl == 1){
                newArmSetpoint(AutoConstants.kArmCubeBot);
            } else if(Positions.gridLvl == 2){
                newArmSetpoint(AutoConstants.kArmCubeMid);
            } else if(Positions.gridLvl == 3){
                newArmSetpoint(AutoConstants.kArmCubeTop);
            }
        }


        if(copilot_2.getRawButtonPressed(OIConstants.kCP2LvlBot)){
            Positions.gridLvl = 1;
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2LvlMid)){
            Positions.gridLvl = 2;
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2LvlTop)){
            Positions.gridLvl = 3;
        }


        if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos1)){
            Positions.gridNumber = 1;
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos2)){
            Positions.gridNumber = 2;
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos3)){
            Positions.gridNumber = 3;
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos4)){
            Positions.gridNumber = 4;
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos5)){
            Positions.gridNumber = 5;
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos6)){
            Positions.gridNumber = 6;
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos7)){
            Positions.gridNumber = 7;
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos8)){
            Positions.gridNumber = 8;
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos9)){
            Positions.gridNumber = 9;
        }

    
        if (driver.getCircleButton()) 
            runCollector(0.2);
        else if (driver.getSquareButton())
            runCollector(-0.35);
        else
            runCollector(0);

        // m_armPIDController.setReference(m_armSetpoint, CANSparkMax.ControlType.kPosition);
        //double armOutput = m_armPID.calculate(m_armEncoder.getPosition(), m_armSetpoint);
        double armOutput = m_armPID.calculate(m_armEncoder.getPosition(), m_armSetpoint);
        if (armOutput < 0){
            armOutput *= 0.75;
        } 
        m_armMax.set(armOutput);

        SmartDashboard.putNumber("Arm Motor", armOutput);
        SmartDashboard.putNumber("Arm Angle (deg)", m_armEncoder.getPosition());
        SmartDashboard.putNumber("Arm Setpoint (deg)", m_armSetpoint);
  
    }
    
    public void newArmSetpoint(double setpoint) {
        m_armSetpoint = setpoint;
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
