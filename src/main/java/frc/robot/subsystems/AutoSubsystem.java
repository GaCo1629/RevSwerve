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
    private TrajectoryConfig        m_config;
    private ProfiledPIDController   m_thetaController;
        
    public AutoSubsystem(DriveSubsystem robotDrive, GPMSubsystem GPM) {
        this.m_robotDrive = robotDrive;
        this.m_GPM = GPM;
        m_config = new TrajectoryConfig(  AutoConstants.kMaxSpeedMetersPerSecond,AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        m_config.setKinematics(DriveConstants.kDriveKinematics);
        m_thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kHeadingLockConstraints);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }


    public void init() {
        m_robotDrive.resetGyroToZero();
    }

    public SwerveControllerCommand runTrajectory( Trajectory myPath) {
        return new SwerveControllerCommand(
            myPath,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            m_thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);
    }

    public Command getBasicAuto(){

        // Basic trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X 
            new Pose2d( Shared.currentPose.getTranslation(), Rotation2d.fromDegrees(135) ),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(    new Translation2d(13.5, 4.8), 
                        new Translation2d(12.5, 4.8),
                        new Translation2d(11.5, 4.8),
                        new Translation2d(10.5, 4.2)
                        ),
            new Pose2d(10.5, 3.5, new Rotation2d(0)),
            m_config);

        // Run path following command, then stop at the end.
        return Commands.sequence(
            m_GPM.newArmSetpointCmd(GPMConstants.kArmCubeTop),
            Commands.waitUntil(Shared.inPosition),
            m_GPM.runCollectorCmd(GPMConstants.kCubeEjectPower),
            Commands.waitSeconds(1),
            m_GPM.runCollectorCmd(0),
            m_GPM.newArmSetpointCmd(GPMConstants.kArmHome),
            Commands.waitUntil(Shared.inPosition), 
            runTrajectory(exampleTrajectory),           
            m_robotDrive.stopCmd(),
            m_robotDrive.lockCurrentHeadingCmd()
        );

    }

}
