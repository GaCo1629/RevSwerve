package frc.robot.subsystems;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Shared;
import frc.robot.Commands.Axial;
import frc.robot.Commands.Yaw;
import frc.robot.Commands.Balance;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GPMConstants;

public class AutoSubsystem extends SubsystemBase {  
     
    private DriveSubsystem          m_robotDrive ;
    private GPMSubsystem            m_GPM; 
    private TrajectoryConfig        m_fastConfig;

    private PIDController           m_xController;
    private PIDController           m_yController;    
    private PIDController           m_hController;

    private final PathConstraints   m_pathConstraints = new PathConstraints(AutoConstants.kMaxSpeedMPS * 0.6, AutoConstants.kMaxAccelerationMPS2 / 2.0);
    
    private final   int             m_numAutos = 9;
    private final   String[]        m_autoNames = {
                                    "WALL CUBE MOBILITY ENGAGE", 
                                    "WALL CUBE MOBILITY",
                                    "CENTER CUBE MOBILITY ENGAGE (def)", 
                                    "CENTER CUBE ENGAGE", 
                                    "FEEDER CUBE MOBILITY ENGAGE",
                                    "FEEDER CUBE MOBILITY",
                                    "FEEDER CONE MOBILITY CONE",
                                    "CUBE", 
                                    "CONE" };

    // Add commands to the autonomous command chooser
    private final SendableChooser<Integer> m_chooser = new SendableChooser<>();
          
    public AutoSubsystem(DriveSubsystem robotDrive, GPMSubsystem GPM) {
        this.m_robotDrive = robotDrive;
        this.m_GPM = GPM;
        
        m_fastConfig = new TrajectoryConfig(  AutoConstants.kMaxSpeedMPS * 0.6, AutoConstants.kMaxAccelerationMPS2);
        m_fastConfig.setKinematics(DriveConstants.kDriveKinematics);
        m_fastConfig.setReversed(true);

        m_xController = new PIDController(AutoConstants.kPXController, 0, 0);
        m_yController = new PIDController(AutoConstants.kPYController, 0, 0);
        m_hController = new PIDController(AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController);
        m_hController.enableContinuousInput(-Math.PI, Math.PI);

        // Put the chooser on the dashboard
        SmartDashboard.putData(m_chooser);  
        //m_chooser.setDefaultOption("Choose Auto: Score CUBE", 0);
        for (int m = 0; m < m_numAutos; m++) {
            if (m == 2)
                m_chooser.setDefaultOption(m_autoNames[m], m);
             else
                m_chooser.addOption(m_autoNames[m], m);
        }
    }

    public void init() {
        m_robotDrive.resetGyroToZero();
        m_GPM.liftGPM();
        Shared.useAprilTags = true;
    }

    @Override
    public void periodic() {
        if (m_chooser != null) {
            int selected = m_chooser.getSelected();
            SmartDashboard.putString("Auto Mode", String.format("(%d) %s", selected, m_autoNames[selected] ));  
        } 
    }

    public Command getAutonomousCommand() {
        switch (m_chooser.getSelected()) {
                     
            case 0:
            return getWallCubeMobilityEngage();

            case 1:
            return getWallCubeMobility();
                
            case 2:
            default:
            return getCenterCubeMobilityEngage();
        
            case 3:
            return getCenterCubeEngage();

            case 4:
            return getFeederCubeMobilityEngage();
                    
            case 5:
            return getFeederCubeMobility();

            case 6:
            return getFeederConeMobilityCone();

            case 7:
            return getCube();
            
            case 8:
            return getCone();
        }
    }
    
    /***
     * 
     * @param myPathName
     * @param SetStartPosition
     * @param useAprilTag
     * @return Command
     */
    public Command runTrajectoryCmd(String myPathName, boolean SetStartPosition, boolean useAprilTag) {

        myPathName = ((DriverStation.getAlliance() == Alliance.Red) ? "Red-" : "Blue-") + myPathName ;
        PathPlannerTrajectory myPath = PathPlanner.loadPath(myPathName, m_pathConstraints);
        
        return Commands.sequence(
            m_robotDrive.useAprilTagsCmd(useAprilTag),
            new InstantCommand(() -> {
                // Reset odometry to planned starting position if we don't know where we are
                if (SetStartPosition) {
                    m_robotDrive.resetOdometry(myPath.getInitialHolonomicPose());
                }
            }),
            new PPSwerveControllerCommand(
                myPath,
                m_robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,
                m_xController,
                m_yController,
                m_hController,
                m_robotDrive::setModuleStates,
                false,   // Do not mirror for red
                m_robotDrive)
        );
    }

    public Command scoreCubeLevel3Cmd() {
        return Commands.sequence(
            m_robotDrive.setStaightCmd(),
            m_GPM.newArmSetpointCmd(GPMConstants.kArmCubeTop),
            Commands.waitUntil(Shared.inPosition),
            m_GPM.runCollectorCmd(GPMConstants.kCubeEjectPower),
            Commands.waitSeconds(1),
            m_GPM.runCollectorCmd(0),
            m_GPM.newArmSetpointCmd(GPMConstants.kArmHome)
        );   
    }

    public Command scoreConeLevel3Cmd() {
        return Commands.sequence(
            m_robotDrive.setStaightCmd(),
            m_GPM.runCollectorCmd(GPMConstants.kConeCollectPower),
            new Axial(m_robotDrive, -0.5, 1.0),
            m_GPM.newArmSetpointCmd(GPMConstants.kArmConeTop),
            Commands.waitUntil(Shared.inPosition),
            m_GPM.runCollectorCmd(GPMConstants.kConeHoldPower),
            new Axial(m_robotDrive, 0.35, 0.5),
            Commands.waitSeconds(0.5),
            m_GPM.runCollectorCmd(GPMConstants.kConeEjectPower),
            Commands.waitSeconds(1),
            m_GPM.runCollectorCmd(0)
        );   
    }

    public Command scoreConeLevel1Cmd() {
        return Commands.sequence(
            m_robotDrive.setStaightCmd(),
            m_GPM.runCollectorCmd(GPMConstants.kConeCollectPower),
            Commands.waitSeconds(0.5),
            m_GPM.runCollectorCmd(0)
        );   
    }

    // ================================================================================================
    public Command getCube(){
        // Just score
        return Commands.sequence(
            scoreCubeLevel3Cmd(),
            Commands.repeatingSequence(m_robotDrive.stopCmd())
        );
    }

    // ================================================================================================
    public Command getCone(){
        // Just score
        return Commands.sequence(
            scoreConeLevel3Cmd(),
            new Axial(m_robotDrive, -0.1, 1.0),
            m_GPM.newArmSetpointCmd(GPMConstants.kArmHome),
            Commands.repeatingSequence(m_robotDrive.stopCmd())
        );
    }

    // ================================================================================================
    public Command getWallCubeMobilityEngage(){

        // Run path following command, then stop at the end.
        return Commands.sequence(
            scoreCubeLevel3Cmd(),
            runTrajectoryCmd("Wall-Engage", true, true),    
            Commands.waitSeconds(0.25),
            Commands.waitUntil(Shared.inPosition), 
            new Balance(m_robotDrive, true),   // balance the robot driving forward 
            Commands.repeatingSequence(m_robotDrive.setXCmd())
        );
    }


    // ================================================================================================
    public Command getWallCubeMobility(){

       // Run path following command, then stop at the end.
        return Commands.sequence(
            scoreCubeLevel3Cmd(),
            Commands.waitUntil(Shared.inPosition), 
            runTrajectoryCmd("Wall-Mobility", true, true),   
            new Yaw(m_robotDrive, (DriverStation.getAlliance() == Alliance.Red) ? Math.PI : 0),
            Commands.repeatingSequence(m_robotDrive.stopCmd())
        );
    }

    // ================================================================================================
    public Command getCenterCubeMobilityEngage(){
        // Run path following command, then stop at the end.
        return Commands.sequence(
            scoreCubeLevel3Cmd(),
            new Axial(m_robotDrive, -4.3, DriveConstants.kRampOverSpeed * DriveConstants.kMaxSpeedMetersPerSecond),
            new Balance(m_robotDrive, true),   // balance the robot in fwd
            Commands.repeatingSequence(m_robotDrive.setXCmd())
        );
    }

    // ================================================================================================
    public Command getCenterCubeEngage(){
        // Run path following command, then stop at the end.
        return Commands.sequence(
            scoreCubeLevel3Cmd(),
            Commands.waitUntil(Shared.inPosition), 
            new Balance(m_robotDrive, false),   // balance the robot in rev
            Commands.repeatingSequence(m_robotDrive.setXCmd())
        );
    }

    // ================================================================================================
    public Command getFeederCubeMobilityEngage(){
     
        // Run path following command, then stop at the end.
        return Commands.sequence(
            scoreCubeLevel3Cmd(),
            runTrajectoryCmd("Feeder-Engage", true, false),          
            Commands.waitUntil(Shared.inPosition), 
            Commands.waitSeconds(0.25),
            new Balance(m_robotDrive, true),   // balance the robot forward 
            Commands.repeatingSequence(m_robotDrive.setXCmd())
        );

    }

    // ================================================================================================
    public Command getFeederCubeMobility(){

        // Run path following command, then stop at the end.
        return Commands.sequence(
            scoreCubeLevel3Cmd(),
            //Commands.waitUntil(Shared.inPosition), 
            runTrajectoryCmd("Feeder-Mobility", true, false), 
            new Yaw(m_robotDrive, (DriverStation.getAlliance() == Alliance.Red) ? Math.PI : 0),
            Commands.repeatingSequence(m_robotDrive.stopCmd())
        );
    }
 
    // ================================================================================================
    public Command getFeederConeMobilityCone(){

        // Run path following command, then stop at the end.
        return Commands.sequence(
            scoreConeLevel1Cmd(),
            runTrajectoryCmd("Feeder-Mobility-Pickup", true, false), 
            new Yaw(m_robotDrive, (DriverStation.getAlliance() == Alliance.Red) ? Math.toRadians(-170) : 10),
            m_GPM.newArmSetpointCmd(GPMConstants.kArmConeGround),
            Commands.waitSeconds(0.25),
            m_GPM.lowerGPMCmd(),
            Commands.waitUntil(Shared.inPosition),
            Commands.waitSeconds(0.5),
            m_GPM.runCollectorCmd(GPMConstants.kConeCollectPower),
            new Axial(m_robotDrive, 0.6, 0.55),
            m_GPM.liftGPMCmd(),
            new Axial(m_robotDrive, -0.6, 0.55),
            m_GPM.newArmSetpointCmd(GPMConstants.kArmHome),

            runTrajectoryCmd("Feeder-Mobility-Place", true, false), 
            new Yaw(m_robotDrive, (DriverStation.getAlliance() == Alliance.Red) ? Math.PI : 0),
            Commands.repeatingSequence(m_robotDrive.stopCmd())
        );
    }
 
//===================================================================================================

}
