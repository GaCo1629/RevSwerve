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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    private CANSparkMax m_armRightMax = new CANSparkMax(GPMConstants.kArmRightCanId, MotorType.kBrushless);
    private CANSparkMax m_armLeftMax = new CANSparkMax(GPMConstants.kArmLeftCanId, MotorType.kBrushless);
    private CANSparkMax m_collectorMax = new CANSparkMax(GPMConstants.kCollectorCanId, MotorType.kBrushless);
    private AbsoluteEncoder m_armEncoder;
    private Spark blinkyLEDs = new Spark(GPMConstants.kLEDpwmID); 

    private double m_armSetpoint ;
    private ProfiledPIDController m_armPID ;

    private double collectorHoldingPower = 0;

    private double plannedGroundPosition = GPMConstants.kArmConeGround;
    private double plannedFeederPosition = GPMConstants.kArmConeFeeder;
         
    public GPMSubsystem(PS4Controller driver, Joystick copilot_1, Joystick copilot_2) {
        this.driver = driver;
        this.copilot_1 = copilot_1;
        this.copilot_2 = copilot_2;

        m_armPID = new ProfiledPIDController(GPMConstants.kArmP, GPMConstants.kArmI, GPMConstants.kArmD,
                                            new Constraints(GPMConstants.kMaxArmVelocity, GPMConstants.kMaxArmAcceleration));
        m_armPID.setIntegratorRange(-GPMConstants.kArmMaxI, GPMConstants.kArmMaxI);
       
        m_armEncoder = m_armRightMax.getAbsoluteEncoder(Type.kDutyCycle);  
        //blinkyLEDs.set(DriverStation.getAlliance() == Alliance.Blue ? GPMConstants.kBlueColor : GPMConstants.kRedColor);
        liftGPM();
    }

    @Override
    public void periodic() {
        runGPMPID();   
        SmartDashboard.putBoolean("Scoring", Shared.cone);
        SmartDashboard.putNumber("Collector Current", m_collectorMax.getOutputCurrent() );
    }

    public void init() {
        newArmSetpoint(m_armEncoder.getPosition());
        m_armPID.reset(m_armEncoder.getPosition());  
        for (int i=1; i < 10; i++) {
            copilot_1.getRawButtonPressed(i);
            copilot_2.getRawButtonPressed(i);
            copilot_1.getRawButtonReleased(i);
            copilot_2.getRawButtonReleased(i);
            
        } 
        blinkyLEDs.set(DriverStation.getAlliance() == Alliance.Blue ? GPMConstants.kBlueColor : GPMConstants.kRedColor);
    }
    
    /**
     * This method is used to control the arm during teleop
     */
    public void teleopRun() {
        
        // Lift and Lower the frame manually
        if (driver.getRawButtonPressed(13) ) {
            if (Shared.liftDown) {
                liftGPM();
            } else {
                lowerGPM();
            }
        }

        // if the driver is trying to spin, lower the arm
        if (driver.getR1Button() || copilot_1.getRawButtonPressed(OIConstants.kCP1Retract)) {
            newArmSetpoint(GPMConstants.kArmHome);    
        }
    
        // Extend and Retract the arm    
        if (driver.getTriangleButtonPressed() && (m_armSetpoint < GPMConstants.kArmMax)) 
            newArmSetpoint(m_armSetpoint + 0.01);
        else if (driver.getCrossButtonPressed() && (m_armSetpoint > GPMConstants.kArmHome))
            newArmSetpoint(m_armSetpoint - 0.01);


        // Look to see if we are Starting or finishing a ground pickup
        if (copilot_1.getRawButtonPressed(OIConstants.kCP1ExtendGround)) {
            setArmGroundPosition();
            lowerGPM();    
        } else if (copilot_1.getRawButtonReleased(OIConstants.kCP1ExtendGround)) {
            liftGPM();    
            newArmSetpoint(GPMConstants.kArmHome);
        } 
        
        // Look to see if we are Starting or finishing a Feeder pickup
        if (copilot_1.getRawButtonPressed(OIConstants.kCP1ExtendFeeder)) {
            setArmFeederPosition();
        } else if (copilot_1.getRawButtonReleased(OIConstants.kCP1ExtendFeeder)) {
            newArmSetpoint(GPMConstants.kArmHome);
        } 
        
        
        // Look for copilot selecting cone or cube for ground pickup
        if (copilot_1.getRawButtonPressed(OIConstants.kCP1GroundCone)) {
            setArmGroundPosition(true);
        } else if (copilot_1.getRawButtonPressed(OIConstants.kCP1GroundCube)) {
            setArmGroundPosition(false);
        }

        // Look for copilot selecting cone or cube for feeder pickup
        if (copilot_1.getRawButtonPressed(OIConstants.kCP1FeederCone)) {
            setArmFeederPosition(true);
        } else if (copilot_1.getRawButtonPressed(OIConstants.kCP1FeederCube)) {
            setArmFeederPosition(false);
        }

                    
        // Grid Scoring
        if (copilot_2.getRawButtonPressed(OIConstants.kCP2LvlBot)){
            setArmScorePosition(1);
        } else if (copilot_2.getRawButtonPressed(OIConstants.kCP2LvlMid)){
            setArmScorePosition(2);
        } else if (copilot_2.getRawButtonPressed(OIConstants.kCP2LvlTop)){
            setArmScorePosition(3);
        } 


        // Look to see if we are finishing a Scoring cycle
        if (copilot_2.getRawButtonReleased(OIConstants.kCP2LvlBot) || 
            copilot_2.getRawButtonReleased(OIConstants.kCP2LvlMid) || 
            copilot_2.getRawButtonReleased(OIConstants.kCP2LvlTop)) {
            SmartDashboard.putString("release", "Yes");
            newArmSetpoint(GPMConstants.kArmHome);
        } 

        
        // Look for cone scoring position
        /* 
        if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos1)){
            setGridTargetPosition(1);
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos2)){
            setGridTargetPosition(2);
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos3)){
            setGridTargetPosition(3);
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos4)){
            setGridTargetPosition(4);
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos5)){
            setGridTargetPosition(5);
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos6)){
            setGridTargetPosition(6);
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos7)){
            setGridTargetPosition(7);
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos8)){
            setGridTargetPosition(8);
        } else if(copilot_2.getRawButtonPressed(OIConstants.kCP2Pos9)){
            setGridTargetPosition(9);
        } 
        */
    

        // run the collector.  Hold onto object once it's grabbed
        if (driver.getCircleButton()) {
            if (Shared.cone) {
                runCollector(GPMConstants.kConeCollectPower);
                collectorHoldingPower = GPMConstants.kConeHoldPower;
            } else {
                runCollector(GPMConstants.kCubeCollectPower);
                collectorHoldingPower = GPMConstants.kCubeHoldPower;
            }
            Shared.haveScoringElement = true;
        } else if (driver.getSquareButton()) {
            if (Shared.cone) {
                runCollector(GPMConstants.kConeEjectPower);
                collectorHoldingPower = 0;
            } else {
                runCollector(GPMConstants.kCubeEjectPower);
                collectorHoldingPower = 0;
            }
            Shared.haveScoringElement = false;
        } else {
            runCollector(collectorHoldingPower);
        }    

        // Set the collector color
        blinkyLEDs.set(Shared.cone ? GPMConstants.kConeColor : GPMConstants.kCubeColor);
   
    }

    void setArmGroundPosition(boolean isCone) {
        Shared.cone = isCone;
        setArmGroundPosition();
    }

    void setArmGroundPosition() {
        if (Shared.cone) {
            plannedGroundPosition = GPMConstants.kArmConeGround;
        } else {
            plannedGroundPosition = GPMConstants.kArmCubeGround;
        }

        if (copilot_1.getRawButton(OIConstants.kCP1ExtendGround)) {
            newArmSetpoint(plannedGroundPosition);        
        }
    }

    void setArmFeederPosition(boolean isCone) {
        Shared.cone = isCone;
        setArmFeederPosition();
    }

    void setArmFeederPosition() {
        if (Shared.cone) {
            plannedFeederPosition = GPMConstants.kArmConeFeeder;
        } else {
            plannedFeederPosition = GPMConstants.kArmCubeFeeder;
        }

        if (copilot_1.getRawButton(OIConstants.kCP1ExtendFeeder)) {
            newArmSetpoint(plannedFeederPosition);        
        }
    }

    void setArmScorePosition(int gridLevel) {
        Shared.gridLevel = gridLevel;
        switch (Shared.gridLevel) {
            case 1:
            if (Shared.cone) 
                newArmSetpoint(GPMConstants.kArmConeBot);
            else
                newArmSetpoint(GPMConstants.kArmCubeBot);
            break;

            case 2:
            if (Shared.cone) 
                newArmSetpoint(GPMConstants.kArmConeMid);
            else
                newArmSetpoint(GPMConstants.kArmCubeMid);
            break;
                        
            case 3:
            if (Shared.cone) 
                newArmSetpoint(GPMConstants.kArmConeTop);
            else
                newArmSetpoint(GPMConstants.kArmCubeTop);
            break;
        }                        
    }

    public void runGPMPID() {
        // Run the arm PID and apply the velocity profile.
        Shared.armPosition = m_armEncoder.getPosition();
        double armOutput = m_armPID.calculate(Shared.armPosition, m_armSetpoint);
        
        /*
        if (armOutput < 0){
            armOutput *= 0.75;
        } 
        */

        // lock arm against backstop
        if ((m_armSetpoint <= GPMConstants.kArmBackstop) && (Shared.armPosition < GPMConstants.kArmBackstopTrigger)) {
            armOutput = GPMConstants.kArmBackPower;
        } 

        // Send power to arm;
        runArm(armOutput);
        Shared.armInPosition = (Math.abs(Shared.armPosition - m_armSetpoint) < 0.05);

        SmartDashboard.putNumber("Right Arm Motor", m_armRightMax.get());
        SmartDashboard.putNumber("Left Arm Motor", m_armLeftMax.get());
        SmartDashboard.putNumber("Arm Angle (deg)", Shared.armPosition);
        SmartDashboard.putNumber("Arm Setpoint (deg)", m_armSetpoint);
        SmartDashboard.putBoolean("Arm In Position", Shared.armInPosition);
    }

    public void setGridTargetPosition(int position) {

        // Save position number and calculate XY position
        double X,Y,H;
        Shared.gridNumber = position;
        if (DriverStation.getAlliance() == Alliance.Red) {
            X = NavConstants.redGridX;        
            Y = NavConstants.redGridY[position - 1];
            H = 0;        
        } else {
            X = NavConstants.blueGridX;        
            Y = NavConstants.blueGridY[position - 1];
            H = Math.PI;        
        }
        Shared.targetPose = new Pose2d(X, Y, new Rotation2d(H));
        Shared.targetPoseSet = true;
        
    }
    
    public void setFeederTargetPosition(boolean rightSide) {
        // Save position number and calculate XY position
        double X,Y,H;
        if (DriverStation.getAlliance() == Alliance.Red) {
            X = NavConstants.redFeederX;        
            Y = NavConstants.redFeederY[rightSide ? 1 : 0];
            H = Math.PI;        
        } else {
            X = NavConstants.blueFeederX;        
            Y = NavConstants.blueFeederY[rightSide ? 1 : 0];
            H = 0;        
        }
        Shared.targetPose = new Pose2d(X, Y, new Rotation2d(H));
        Shared.targetPoseSet = true;
    }
    
    public void newArmSetpoint(double setpoint) {
        m_armSetpoint = setpoint;
        Shared.armSetpoint = setpoint;
        Shared.armInPosition = false;
    }
    public CommandBase newArmSetpointCmd(double setpoint) {return this.runOnce(() -> newArmSetpoint(setpoint));}


    /** Lift the Arm. Support */
    public void liftGPM() {
      m_liftSolenoid.set(kForward);
      Shared.liftDown = false;
    }
    public CommandBase liftGPMCmd() {return this.runOnce(() -> liftGPM());}


    /** Lower the arm. Support*/
    public void lowerGPM() {
        m_liftSolenoid.set(kReverse);
        Shared.liftDown = true;
    }
    public CommandBase lowerGPMCmd() {return this.runOnce(() -> lowerGPM());}


    public void runArm(double speed) {
        m_armRightMax.set(speed);
        m_armLeftMax.set(-speed);
    }
    
    public void runCollector(double speed) {
        m_collectorMax.set(speed);
    }
    public CommandBase runCollectorCmd(double speed) {return this.runOnce(() -> runCollector(speed));}

}
