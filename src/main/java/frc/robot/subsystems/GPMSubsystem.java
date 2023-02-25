package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import frc.robot.Shared;
import frc.robot.Constants.GPMConstants;
import frc.robot.Constants.NavConstants;
import frc.robot.Constants.OIConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Game-piece Scoring Mechanism
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

    private double m_armSetpoint ;
    private ProfiledPIDController m_armPID ;

    private double collectorHoldingPower = 0;
       
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
        
        // Lift and Lower the frame
        if (driver.getL1ButtonPressed()) 
            liftGPM();
        else if (driver.getR1ButtonPressed())
            lowerGPM();

        // Lift and Lower the Arm    
        if (driver.getTriangleButtonPressed() && (m_armSetpoint < GPMConstants.kArmMax)) 
            newArmSetpoint(m_armSetpoint + 0.01);
        else if (driver.getCrossButtonPressed() && (m_armSetpoint > GPMConstants.kArmHome))
            newArmSetpoint(m_armSetpoint - 0.01);

        // Look for cone pickup    
        if (copilot_1.getRawButtonPressed(OIConstants.kCP1Retract)) {
            newArmSetpoint(GPMConstants.kArmHome);
        } else if (copilot_1.getRawButtonPressed(OIConstants.kCP1InCone)) {
            newArmSetpoint(GPMConstants.kArmHumCone);
            Shared.cone = true;
        } else if (copilot_1.getRawButtonPressed(OIConstants.kCP1InCube)) {
            newArmSetpoint(GPMConstants.kArmHumCube);
            Shared.cone = false;
        } else if (copilot_1.getRawButtonPressed(OIConstants.kCP1InGround)) { 
            newArmSetpoint(GPMConstants.kArmGround);
        } else if (copilot_1.getRawButtonPressed(OIConstants.kCP1OutCone)){ 
            Shared.cone = true;
            if(Shared.gridLvl == 1){
                newArmSetpoint(GPMConstants.kArmConeBot);
            } else if(Shared.gridLvl == 2){
                newArmSetpoint(GPMConstants.kArmConeMid);
            } else if(Shared.gridLvl == 3){
                newArmSetpoint(GPMConstants.kArmConeTop);
            }
        }
        else if (copilot_1.getRawButtonPressed(OIConstants.kCP1OutCube)){
            Shared.cone = false;
            if(Shared.gridLvl == 1){
                newArmSetpoint(GPMConstants.kArmCubeBot);
            } else if(Shared.gridLvl == 2){
                newArmSetpoint(GPMConstants.kArmCubeMid);
            } else if(Shared.gridLvl == 3){
                newArmSetpoint(GPMConstants.kArmCubeTop);
            }
        }

        // Look for cone scoring level
        if(copilot_2.getRawButtonPressed(OIConstants.kCP2LvlBot)){
            Shared.gridLvl = 1;
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2LvlMid)){
            Shared.gridLvl = 2;
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2LvlTop)){
            Shared.gridLvl = 3;
        }

        // Look for cone scoring position
        if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos1)){
            setTargetPosition(1);
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos2)){
            setTargetPosition(2);
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos3)){
            setTargetPosition(3);
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos4)){
            setTargetPosition(4);
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos5)){
            setTargetPosition(5);
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos6)){
            setTargetPosition(6);
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos7)){
            setTargetPosition(7);
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos8)){
            setTargetPosition(8);
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos9)){
            setTargetPosition(9);
        }

        if (driver.getRawButtonPressed(13)) {
            Shared.cone = !Shared.cone;
        }

        SmartDashboard.putString("Scoring", Shared.cone ? "Cone" : "Cube");

        // run the collector.  Hold onto object once it's grabbed
        if (driver.getCircleButton()) {
            if (Shared.cone) {
                runCollector(GPMConstants.kConeCollectPower);
                collectorHoldingPower = GPMConstants.kConeHoldPower;
            } else {
                runCollector(GPMConstants.kCubeCollectPower);
                collectorHoldingPower = GPMConstants.kCubeHoldPower;
            }
        } else if (driver.getSquareButton()) {
            if (Shared.cone) {
                runCollector(GPMConstants.kConeEjectPower);
                collectorHoldingPower = 0;
            } else {
                runCollector(GPMConstants.kCubeEjectPower);
                collectorHoldingPower = 0;
            }
        } else {
            runCollector(collectorHoldingPower);
        }    

        // Run the arm PID and apply the velocity profile.
        double armOutput = m_armPID.calculate(m_armEncoder.getPosition(), m_armSetpoint);
        if (armOutput < 0){
            armOutput *= 0.75;
        } 

        // lock arm against backstop
        if ((m_armSetpoint < GPMConstants.kArmBackstop) && (m_armEncoder.getPosition() < GPMConstants.kArmBackstopTrigger)) {
            armOutput = GPMConstants.kArmBackPower;
        } 

        // Send power to arm;
        m_armMax.set(armOutput); 
        
        SmartDashboard.putNumber("Arm Motor", armOutput);
        SmartDashboard.putNumber("Arm Angle (deg)", m_armEncoder.getPosition());
        SmartDashboard.putNumber("Arm Setpoint (deg)", m_armSetpoint);
    }

    public void setTargetPosition(int position) {
        // Save position number and calculate XY position
        Shared.gridNumber = position;
        if (DriverStation.getAlliance() == Alliance.Red) {
            Shared.targetX = 14.0;        
            Shared.targetY = NavConstants.redYPos[position - 1];
            Shared.targetH = 0;        
        } else {
            Shared.targetX = 2.5;        
            Shared.targetY = NavConstants.redYPos[9 - position];
            Shared.targetH = Math.PI;        
        }
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
