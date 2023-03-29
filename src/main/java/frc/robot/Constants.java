// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever 
 * 
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
   //Vision Constants
   public static final class VisionConstants {
    public static String cameraName = "OV5647";
    static final Transform3d robotToCam =
    new Transform3d(
            new Translation3d(-0.12, 0.0, 0.74),
            new Rotation3d(
                    0, 0,
                    0)); // Cam mounted facing forward, 5 in forward of center, 29 in, up from center.

  }
  
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kSpeedFactor = 0.81;
    public static final double kTurnFactor  = 0.70; 
    public static final double kDPADSpeed   = 0.20; 
    public static final double kRampOverSpeed   = 0.25; 
    public static final double kMinApproachRate = 0.075;

    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 1.8; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
   
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kFrontRightDrivingCanId = 13;
    public static final int kRearLeftDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearLeftTurningCanId = 16;
    public static final int kRearRightTurningCanId = 18;

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedMps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
 
    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 3.0e-5 / ModuleConstants.kDrivingEncoderVelocityFactor; // was  0.04;
    public static final double kDrivingI = 7.0e-7 / ModuleConstants.kDrivingEncoderVelocityFactor;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedMps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;
    public static final double kIZone = 400 * ModuleConstants.kDrivingEncoderVelocityFactor;
    

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kArmMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoPilotController1Port= 1;
    public static final int kCoPilotController2Port= 2;
    public static final double kDriveDeadband = 0.05;
    public static final double kDriveReallyDeadband = 0.2;
    public static final int kDriverGyroReset = 14;
    public static final int kDriverSquareUp = 8;  // Right Trigger as button
    public static final int kCP1Retract = 1;
    public static final int kCP1GroundCone = 2;
    public static final int kCP1GroundCube = 3;
    public static final int kCP1ExtendGround = 4;
    public static final int kCP1FeederCone = 5;
    public static final int kCP1FeederCube = 6;
    public static final int kCP1ExtendFeeder = 7;
    public static final int kCP2Pos1 = 1;
    public static final int kCP2Pos2 = 2;
    public static final int kCP2Pos3 = 3;
    public static final int kCP2Pos4 = 4;
    public static final int kCP2Pos5 = 5;
    public static final int kCP2Pos6 = 6;
    public static final int kCP2Pos7 = 7;
    public static final int kCP2Pos8 = 8;
    public static final int kCP2Pos9 = 9;
    public static final int kCP2LvlBot = 10;
    public static final int kCP2LvlMid = 11;
    public static final int kCP2LvlTop = 12;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMPS = 3;
    public static final double kMaxAccelerationMPS2 = 3;
    public static final double kMaxAngularSpeedRPS = Math.PI;
    public static final double kMaxAngularAccelerationRPS2 = Math.PI;

    public static final double kPXController = 2;
    public static final double kPYController = 2;
    public static final double kPThetaController = 1;

    public static final double kAutoMaxAngularSpeedRPS = Math.PI * 2;
    public static final double kAutoMaxAngularAccelerationRPS2 = Math.PI * 2 ;

    public static final double kBalanceApproachSpeedMPS = 0.4 ;
   
    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kTranslateControllerConstraints = new TrapezoidProfile.Constraints(3, 1);

    public static final TrapezoidProfile.Constraints kYawControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeedRPS, kMaxAngularAccelerationRPS2);
  
    public static final TrapezoidProfile.Constraints kHeadingLockConstraints = new TrapezoidProfile.Constraints(
      kAutoMaxAngularSpeedRPS, kAutoMaxAngularAccelerationRPS2);
  
    public static final double kPHeadingLockController = 1.5; // unit gain
    public static final double kIHeadingLockController = 0;
    public static final double kDHeadingLockController = 0; 

    public static final double kPYawController = 6;  // radian gain
    public static final double kIYawController = 0;
    public static final double kDYawController = 0; 

    public static final double kNotRotating = 0.5;  // degrees per second was 0.5
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class GPMConstants {
    public static final int kArmRightCanId = 19; 
    public static final int kArmLeftCanId = 21;
    public static final int kCollectorCanId = 20; 
    public static final int kGPMSolenoidModule = 2;
    public static final int[] kGPMSolenoidPorts = new int[] {15, 0}; // Up, Down
    
    public static final double kArmP = 3.5;
    public static final double kArmI = 0;
    public static final double kArmD = 0;
    public static final double kArmMaxI = 0.10;

    public static final double kMaxArmVelocity = 0.3 ;
    public static final double kMaxArmAcceleration = 1.0 ;

   
    public static final double kConeCollectPower  = -0.75 ;
    public static final double kConeAutoEjectPower = -0.3 ;
    public static final double kConeHoldPower     = -0.15 ; 
    public static final double kConeEjectPower    =  0.2 ;
    public static final double kCubeCollectPower  =  0.4 ;
    public static final double kCubeHoldPower     =  0.15 ;
    public static final double kCubeEjectPower    = -0.2 ;
    
    public static final double kArmConeBot = 0.23; // height = 19.25"
    public static final double kArmConeMid = 0.46; // height = 40"
    public static final double kArmConeTop = 0.57; // height = 53.5"
    public static final double kArmCubeBot = 0.26; // height = 21.75"
    public static final double kArmCubeMid = 0.48; // height = 42"
    public static final double kArmCubeTop = 0.55; // height = 51.25"
    public static final double kArmConeFeeder = 0.55; // height in = 51.125"
    public static final double kArmCubeFeeder = 0.54; // height in = 49.5"
    public static final double kArmHome = 0.16;
    public static final double kArmGround = 0.36;
    public static final double kArmConeGround = 0.38;
    public static final double kArmCubeGround = 0.34;
    public static final double kArmMax  = 0.6;
    
    public static final double kArmBackstop = 0.16;   
    public static final double kArmBackstopTrigger = 0.18;   
    public static final double kArmSafeToSpinn = 0.23;   
    public static final double kArmBackPower = -0.04;

    public static final int    kLEDpwmID  = 9 ;
    public static final double kCubeColor = 0.91;
    public static final double kConeColor = 0.63;
    public static final double kRedColor = 0.59;
    public static final double kGreenColor = 0.75;
    public static final double kBlueColor = 0.85;
    public static final double kBlackColor = 0.99;
        
  }

  public static final class NavConstants {
    public static final double [] redGridY  = {1.07,1.07,1.07, 2.75,2.75,2.75, 4.42,4.42,4.42};
    public static final double [] blueGridY = {4.42,4.42,4.42, 2.75,2.75,2.75, 1.07,1.07,1.07};
    public static final double [] redFeederY  = {6.0, 7.0};
    public static final double [] blueFeederY  = {7.0,6.0};
    public static final double redGridX     = 14.0;
    public static final double blueGridX    =  2.5;
    public static final double redFeederX   =  2.0;
    public static final double blueFeederX  = 14.54;
    public static final double redFeederDir  = Math.PI;
    public static final double blueFeederDir = 0;
  }
}
 