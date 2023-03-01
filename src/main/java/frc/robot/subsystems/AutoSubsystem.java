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
        
    public AutoSubsystem(DriveSubsystem robotDrive, GPMSubsystem GPM) {
        this.m_robotDrive = robotDrive;
        this.m_GPM = GPM;
        
        m_fastConfig = new TrajectoryConfig(  AutoConstants.kMaxSpeedMetersPerSecond / 2,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        m_fastConfig.setKinematics(DriveConstants.kDriveKinematics);

        m_slowConfig = new TrajectoryConfig(  AutoConstants.kMaxSpeedMetersPerSecond / 5, 
                                                AutoConstants.kMaxAccelerationMetersPerSecondSquared / 2);
        m_slowConfig.setKinematics(DriveConstants.kDriveKinematics);

        m_slowRevConfig = new TrajectoryConfig(  AutoConstants.kMaxSpeedMetersPerSecond / 5, 
        AutoConstants.kMaxAccelerationMetersPerSecondSquared / 2);
        m_slowRevConfig.setKinematics(DriveConstants.kDriveKinematics);
        m_slowRevConfig.setReversed(true);

        m_xController = new PIDController(AutoConstants.kPXController, 0, 0);
        m_yController = new PIDController(AutoConstants.kPYController, 0, 0);

        m_hController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kHeadingLockConstraints);
        m_hController.enableContinuousInput(-Math.PI, Math.PI);
    }


    public void init() {
        m_robotDrive.resetGyroToZero();
        m_GPM.liftGPM();
        Shared.useAprilTags = true;
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

    public Command getRed2Auto(){

        // Protect in case we havent seen the target yet
        if (Math.abs(Shared.currentPose.getX()) <  0.5) {
            Shared.currentPose = new Pose2d(14.5, 2.74, new Rotation2d(0));
        }

        // Basic trajectory to follow. All units in meters.
        Trajectory red2ToUpRamp = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X 
            Shared.currentPose, 
            List.of(new Translation2d(13.5, 2.74)),
            new Pose2d(12.2, 2.74, new Rotation2d(0)),
            m_slowRevConfig);
    
            // Run path following command, then stop at the end.
        return Commands.sequence(
            m_GPM.newArmSetpointCmd(GPMConstants.kArmCubeTop),
            Commands.waitUntil(Shared.inPosition),
            m_GPM.runCollectorCmd(GPMConstants.kCubeEjectPower),
            Commands.waitSeconds(1),
            m_GPM.runCollectorCmd(0),
            m_GPM.newArmSetpointCmd(GPMConstants.kArmHome),
            Commands.waitUntil(Shared.inPosition), 
            runTrajectory(red2ToUpRamp),           
            Commands.repeatingSequence(m_robotDrive.setXCmd())
        );

    }

    public Command getRed3Auto(){

        // Protect in case we havent seen the target yet
        if (Math.abs(Shared.currentPose.getX()) <  0.5) {
            Shared.currentPose = new Pose2d(14.5, 4.4, new Rotation2d(0.0));
        }

        // Basic trajectory to follow. All units in meters.
        Trajectory red3ToOutsideRamp = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X 
            new Pose2d( Shared.currentPose.getTranslation(), Rotation2d.fromDegrees(180) ),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(    new Translation2d(13.5, 4.4), 
                        new Translation2d(12.5, 4.4),
                        new Translation2d(11.5, 4.4),
                        new Translation2d(10.5, 3.9)
                        ),
            new Pose2d(10.5, 2.74, new Rotation2d(0)),
            m_fastConfig);

        Trajectory red3UpRampFromOutsideRamp = TrajectoryGenerator.generateTrajectory(
            // Start From outside the ramp 
            new Pose2d(10.6, 2.7, new Rotation2d(0)),
            List.of(new Translation2d(12.45, 2.7)),
            new Pose2d(12.5, 2.7, new Rotation2d(0)),
            m_slowConfig);
    
            // Run path following command, then stop at the end.
        return Commands.sequence(
            m_GPM.newArmSetpointCmd(GPMConstants.kArmCubeTop),
            Commands.waitUntil(Shared.inPosition),
            m_GPM.runCollectorCmd(GPMConstants.kCubeEjectPower),
            Commands.waitSeconds(1),
            m_GPM.runCollectorCmd(0),
            m_GPM.newArmSetpointCmd(GPMConstants.kArmHome),
            Commands.waitUntil(Shared.inPosition), 
            runTrajectory(red3ToOutsideRamp),           
            // m_robotDrive.stopCmd(),
            // m_robotDrive.useAprilTagsCmd(false),
            runTrajectory(red3UpRampFromOutsideRamp),
            Commands.repeatingSequence(m_robotDrive.setXCmd())
        );

    }

}
