package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Shared;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GPMConstants;

public class AutoSubsystem extends SubsystemBase {  
     
    private DriveSubsystem          m_robotDrive ;
    private GPMSubsystem            m_GPM; 
    private TrajectoryConfig        m_slowConfig;
    private TrajectoryConfig        m_slowRevConfig;
    private TrajectoryConfig        m_fastConfig;

    private PIDController           m_xController;
    private PIDController           m_yController;    
    private ProfiledPIDController   m_hController;
    
    private final   int             m_numAutos = 6;
    private final   String[]        m_autoNames = {"DO NOTHING", 
                                    "WALL start with RAMP", 
                                    "CENTER start with RAMP", 
                                    "FEEDER Start with RAMP",
                                    "Wall start no ramp",
                                    "feeder start no ramp" };

    // Add commands to the autonomous command chooser
    private final SendableChooser<Integer> m_chooser = new SendableChooser<>();
          
    public AutoSubsystem(DriveSubsystem robotDrive, GPMSubsystem GPM) {
        this.m_robotDrive = robotDrive;
        this.m_GPM = GPM;
        
        m_fastConfig = new TrajectoryConfig(  AutoConstants.kMaxSpeedMetersPerSecond / 2,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        m_fastConfig.setKinematics(DriveConstants.kDriveKinematics);

        m_slowConfig = new TrajectoryConfig(  AutoConstants.kMaxSpeedMetersPerSecond / 3, 
                                                AutoConstants.kMaxAccelerationMetersPerSecondSquared / 2);
        m_slowConfig.setKinematics(DriveConstants.kDriveKinematics);

        m_slowRevConfig = new TrajectoryConfig(  AutoConstants.kMaxSpeedMetersPerSecond / 3, 
        AutoConstants.kMaxAccelerationMetersPerSecondSquared / 2);
        m_slowRevConfig.setKinematics(DriveConstants.kDriveKinematics);
        m_slowRevConfig.setReversed(true);

        m_xController = new PIDController(AutoConstants.kPXController, 0, 0);
        m_yController = new PIDController(AutoConstants.kPYController, 0, 0);

        m_hController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kHeadingLockConstraints);
        m_hController.enableContinuousInput(-Math.PI, Math.PI);

        // Put the chooser on the dashboard
        SmartDashboard.putData(m_chooser);  
        m_chooser.setDefaultOption("Choose an Auto", 0);
        for (int m = 1; m < m_numAutos; m++) {
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
            default:
            return getDoNothing();
            
            case 1:
            return getWallRampAuto();
        
            case 2:
            return getCenterRampAuto();
        
            case 3:
            return getFeederRampAuto();
        
            case 4:
            return getWallNoRampAuto();
        
            case 5:
            return getFeederNoRampAuto();
        }
    }
    
    public SwerveControllerCommand runTrajectory( Trajectory myPath) {
        return new SwerveControllerCommand(
            myPath,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            m_xController,
            m_yController,
            m_hController,
            m_robotDrive::setModuleStates,
            m_robotDrive);
    }

    // ================================================================================================
    public Command getDoNothing(){
        // Run path following command, then stop at the end.
        return Commands.sequence(
            m_GPM.newArmSetpointCmd(GPMConstants.kArmCubeTop),
            Commands.waitUntil(Shared.inPosition),
            m_GPM.runCollectorCmd(GPMConstants.kCubeEjectPower),
            Commands.waitSeconds(1),
            m_GPM.runCollectorCmd(0),
            m_GPM.newArmSetpointCmd(GPMConstants.kArmHome),
            Commands.repeatingSequence(m_robotDrive.stopCmd())
        );
    }

    // ================================================================================================
    public Command getWallRampAuto(){

        Trajectory wallToOutsideRamp;
        Trajectory wallUpRampFromOutsideRamp;

        if (DriverStation.getAlliance() == Alliance.Red) {
            // Protect in case we havent seen the target yet
            if (Math.abs(Shared.currentPose.getX()) <  0.5) {
                Shared.currentPose = new Pose2d(14.5, 0.97, new Rotation2d(0.0));
            }

           // Basic trajectory to follow from RED side. All units in meters.
            wallToOutsideRamp = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X 
                new Pose2d( Shared.currentPose.getTranslation(), Rotation2d.fromDegrees(-150) ),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(    new Translation2d(13.5, 0.87), 
                            new Translation2d(12.5, 0.87),
                            new Translation2d(11.5, 0.87),
                            new Translation2d(10.5, 1.47)
                            ),
                new Pose2d(10.5, 2.7, new Rotation2d(0)),
                m_fastConfig);

            wallUpRampFromOutsideRamp = TrajectoryGenerator.generateTrajectory(
                // Start From outside the ramp 
                new Pose2d(10.6, 2.7, new Rotation2d(0)),
                List.of(new Translation2d(12.45, 2.7)),
                new Pose2d(12.5, 2.7, new Rotation2d(0)),
                m_slowConfig);
        } else {
            // Protect in case we havent seen the target yet
            if (Math.abs(Shared.currentPose.getX()) <  0.5) {
                Shared.currentPose = new Pose2d(2.03, 0.97, new Rotation2d(Math.PI));
            }

            // Basic trajectory to follow from BLUE side. All units in meters.
            wallToOutsideRamp = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X 
                new Pose2d( Shared.currentPose.getTranslation(), Rotation2d.fromDegrees(-30) ),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(    new Translation2d(3.03, 0.87), 
                            new Translation2d(4.03, 0.87),
                            new Translation2d(5.03, 0.87),
                            new Translation2d(6.03, 1.47)
                            ),
                new Pose2d(6.03, 2.7, new Rotation2d(Math.PI)),
                m_fastConfig);

            wallUpRampFromOutsideRamp = TrajectoryGenerator.generateTrajectory(
                // Start From outside the ramp 
                new Pose2d(5.93, 2.7, new Rotation2d(Math.PI)),
                List.of(new Translation2d(4.08, 2.7)),
                new Pose2d(4.03, 2.7, new Rotation2d(Math.PI)),
                m_slowConfig);

        }

            // Run path following command, then stop at the end.
        return Commands.sequence(
            m_GPM.newArmSetpointCmd(GPMConstants.kArmCubeTop),
            Commands.waitUntil(Shared.inPosition),
            m_GPM.runCollectorCmd(GPMConstants.kCubeEjectPower),
            Commands.waitSeconds(1),
            m_GPM.runCollectorCmd(0),
            m_GPM.newArmSetpointCmd(GPMConstants.kArmHome),
            runTrajectory(wallToOutsideRamp),  
            Commands.waitUntil(Shared.inPosition), 
            // m_robotDrive.useAprilTagsCmd(false),
            runTrajectory(wallUpRampFromOutsideRamp),
            Commands.repeatingSequence(m_robotDrive.setXCmd())
        );
    }


    // ================================================================================================
    public Command getWallNoRampAuto(){

        Trajectory wallToOutsideRamp;
        
        if (DriverStation.getAlliance() == Alliance.Red) {
            // Protect in case we havent seen the target yet
            if (Math.abs(Shared.currentPose.getX()) <  0.5) {
                Shared.currentPose = new Pose2d(14.5, 0.97, new Rotation2d(0.0));
            }

           // Basic trajectory to follow from RED side. All units in meters.
            wallToOutsideRamp = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X 
                new Pose2d( Shared.currentPose.getTranslation(), Rotation2d.fromDegrees(-150) ),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(    new Translation2d(13.5, 0.87), 
                            new Translation2d(12.5, 0.87),
                            new Translation2d(11.5, 0.87),
                            new Translation2d(10.5, 1.37)
                            ),
                new Pose2d(10.5, 1.87, new Rotation2d(0)),
                m_fastConfig);

        } else {
            // Protect in case we havent seen the target yet
            if (Math.abs(Shared.currentPose.getX()) <  0.5) {
                Shared.currentPose = new Pose2d(2.03, 0.97, new Rotation2d(Math.PI));
            }

            // Basic trajectory to follow from BLUE side. All units in meters.
               
            wallToOutsideRamp = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X 
                new Pose2d( Shared.currentPose.getTranslation(), Rotation2d.fromDegrees(-30) ),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(    new Translation2d(3.03, 0.87), 
                            new Translation2d(4.03, 0.87),
                            new Translation2d(5.03, 0.87),
                            new Translation2d(6.03, 1.37)
                            ),
                new Pose2d(6.03, 1.87, new Rotation2d(Math.PI)),
                m_fastConfig);

        }

        // Run path following command, then stop at the end.
        return Commands.sequence(
            m_GPM.newArmSetpointCmd(GPMConstants.kArmCubeTop),
            Commands.waitUntil(Shared.inPosition),
            m_GPM.runCollectorCmd(GPMConstants.kCubeEjectPower),
            Commands.waitSeconds(1),
            m_GPM.runCollectorCmd(0),
            m_GPM.newArmSetpointCmd(GPMConstants.kArmHome),
            Commands.waitUntil(Shared.inPosition), 

            //m_robotDrive.useAprilTagsCmd(false),
            runTrajectory(wallToOutsideRamp),  
            Commands.repeatingSequence(m_robotDrive.stopCmd())
        );
    }

    // ================================================================================================
    public Command getCenterRampAuto(){

        Trajectory centerToUpRamp;

        if (DriverStation.getAlliance() == Alliance.Red) {

            // Protect in case we havent seen the target yet
            if (Math.abs(Shared.currentPose.getX()) <  0.5) {
                Shared.currentPose = new Pose2d(14.5, 2.74, new Rotation2d(0));
            }

            // Basic trajectory to follow on RED side. All units in meters.
            centerToUpRamp = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X 
                Shared.currentPose, 
                List.of(new Translation2d(13.5, 2.74)),
                new Pose2d(12.35, 2.7, new Rotation2d(0)),
                m_slowRevConfig);
        } else {

            // Protect in case we havent seen the target yet
            if (Math.abs(Shared.currentPose.getX()) <  0.5) {
                Shared.currentPose = new Pose2d(2.03, 2.74, new Rotation2d(Math.PI));
            }
 
            // Basic trajectory to follow on BLUE side. All units in meters.
            centerToUpRamp = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X 
                Shared.currentPose, 
                List.of(new Translation2d(3.03, 2.74)),
                new Pose2d(4.2, 2.7, new Rotation2d(Math.PI)),
                m_slowRevConfig);
        }
    
        // Run path following command, then stop at the end.
        return Commands.sequence(
            m_GPM.newArmSetpointCmd(GPMConstants.kArmCubeTop),
            Commands.waitUntil(Shared.inPosition),
            m_GPM.runCollectorCmd(GPMConstants.kCubeEjectPower),
            Commands.waitSeconds(1),
            m_GPM.runCollectorCmd(0),
            m_GPM.newArmSetpointCmd(GPMConstants.kArmHome),
            //Commands.waitUntil(Shared.inPosition), 
            runTrajectory(centerToUpRamp),           
            Commands.repeatingSequence(m_robotDrive.setXCmd())
        );
    }

    // ================================================================================================
    public Command getFeederRampAuto(){

        Trajectory feederToOutsideRamp;
        Trajectory feederUpRampFromOutsideRamp;

        if (DriverStation.getAlliance() == Alliance.Red) {

            // Protect in case we havent seen the target yet
            if (Math.abs(Shared.currentPose.getX()) <  0.5) {
                Shared.currentPose = new Pose2d(14.6, 4.5, new Rotation2d(0.0));
            }

            // Basic trajectory to follow in RED. All units in meters.
            feederToOutsideRamp = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X 
                new Pose2d( Shared.currentPose.getTranslation(), Rotation2d.fromDegrees(160) ),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(    new Translation2d(13.5, 4.7), 
                            new Translation2d(12.5, 4.7),
                            new Translation2d(11.5, 4.7),
                            new Translation2d(10.5, 3.9)
                            ),
                new Pose2d(10.5, 2.74, new Rotation2d(0)),
                m_fastConfig);

            feederUpRampFromOutsideRamp = TrajectoryGenerator.generateTrajectory(
                // Start From outside the ramp 
                new Pose2d(10.6, 2.7, new Rotation2d(0)),
                List.of(new Translation2d(12.45, 2.7)),
                new Pose2d(12.5, 2.7, new Rotation2d(0)),
                m_slowConfig);
        } else {

            // Protect in case we havent seen the target yet
            if (Math.abs(Shared.currentPose.getX()) <  0.5) {
                Shared.currentPose = new Pose2d(2.03, 4.5, new Rotation2d(Math.PI));
            }

            // Basic trajectory to follow in BLUE. All units in meters.
            feederToOutsideRamp = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X 
                new Pose2d( Shared.currentPose.getTranslation(), Rotation2d.fromDegrees(30) ),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(    new Translation2d(3.03, 4.7), 
                            new Translation2d(4.03, 4.7),
                            new Translation2d(5.03, 4.7),
                            new Translation2d(6.03, 3.9)
                            ),
                new Pose2d(6.03, 2.74, new Rotation2d(Math.PI)),
                m_fastConfig);

            feederUpRampFromOutsideRamp = TrajectoryGenerator.generateTrajectory(
                // Start From outside the ramp 
                new Pose2d(5.93, 2.7, new Rotation2d(Math.PI)),
                List.of(new Translation2d(4.08, 2.7)),
                new Pose2d(4.03, 2.7, new Rotation2d(Math.PI)),
                m_slowConfig);
        }
            
        // Run path following command, then stop at the end.
        return Commands.sequence(
            m_GPM.newArmSetpointCmd(GPMConstants.kArmCubeTop),
            Commands.waitUntil(Shared.inPosition),
            m_GPM.runCollectorCmd(GPMConstants.kCubeEjectPower),
            Commands.waitSeconds(1),
            m_GPM.runCollectorCmd(0),
            m_GPM.newArmSetpointCmd(GPMConstants.kArmHome),
            runTrajectory(feederToOutsideRamp),           
            // m_robotDrive.useAprilTagsCmd(false),
            Commands.waitUntil(Shared.inPosition), 
            runTrajectory(feederUpRampFromOutsideRamp),
            Commands.repeatingSequence(m_robotDrive.setXCmd())
        );

    }

    // ================================================================================================
    public Command getFeederNoRampAuto(){

        Trajectory feederToOutsideRamp;

        if (DriverStation.getAlliance() == Alliance.Red) {

            // Protect in case we havent seen the target yet
            if (Math.abs(Shared.currentPose.getX()) <  0.5) {
                Shared.currentPose = new Pose2d(14.6, 4.5, new Rotation2d(0.0));
            }

            // Basic trajectory to follow in RED. All units in meters.
            feederToOutsideRamp = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X 
                new Pose2d( Shared.currentPose.getTranslation(), Rotation2d.fromDegrees(160) ),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(    new Translation2d(13.5, 4.7), 
                            new Translation2d(12.5, 4.7),
                            new Translation2d(11.5, 4.7),
                            new Translation2d(10.5, 4.2)
                            ),
                new Pose2d(10.5, 3.7, new Rotation2d(0)),
                m_fastConfig);
        } else {

            if (Math.abs(Shared.currentPose.getX()) <  0.5) {
                Shared.currentPose = new Pose2d(2.03, 4.5, new Rotation2d(Math.PI));
            }

            // Basic trajectory to follow in BLUE. All units in meters.
            feederToOutsideRamp = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X 
                new Pose2d( Shared.currentPose.getTranslation(), Rotation2d.fromDegrees(30) ),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(    new Translation2d(3.03, 4.7), 
                            new Translation2d(4.03, 4.7),
                            new Translation2d(5.03, 4.7),
                            new Translation2d(6.03, 4.2)
                            ),
                new Pose2d(6.03, 3.7, new Rotation2d(Math.PI)),
                m_fastConfig);
        }
            
        // Run path following command, then stop at the end.
        return Commands.sequence(
            m_GPM.newArmSetpointCmd(GPMConstants.kArmCubeTop),
            Commands.waitUntil(Shared.inPosition),
            m_GPM.runCollectorCmd(GPMConstants.kCubeEjectPower),
            Commands.waitSeconds(1),
            m_GPM.runCollectorCmd(0),
            m_GPM.newArmSetpointCmd(GPMConstants.kArmHome),
            Commands.waitUntil(Shared.inPosition), 
            //m_robotDrive.useAprilTagsCmd(false),
            runTrajectory(feederToOutsideRamp),           
            Commands.repeatingSequence(m_robotDrive.stopCmd())
        );
    }

}
