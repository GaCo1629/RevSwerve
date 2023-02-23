// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Positions;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  //private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP); 
  private double gyro2FieldOffset = 0;
  private double gyro2FCDOffset = 0; 

  private SlewRateLimiter m_xLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_yLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  
  private PS4Controller driver;
  private ProfiledPIDController headingLockController;
  private boolean headingLocked = false;
  private double  currentHeading = 0;
  private double  headingSetpoint = 0;  

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(PS4Controller driver, Joystick copilot_1, Joystick copilot_2) {
    this.driver = driver;

    headingLockController = new ProfiledPIDController(AutoConstants.kPHeadingLockController, 0, 0,
    new Constraints(AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond,AutoConstants.kAutoMaxAngularAccelerationRadiansPerSecondSquared));
    headingLockController.enableContinuousInput(-Math.PI, Math.PI);
  }
 
  public void init() {
    setFieldOffsets();
    lockCurrentHeading();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromRadians(getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    if(driver.getRawButtonPressed(OIConstants.kDriverGyroReset)){
      resetGyroToZero();
      lockCurrentHeading();
    }

    SmartDashboard.putString("DrivePeriodic", m_odometry.getPoseMeters().toString());
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putNumber("Level", Positions.gridLvl);
    SmartDashboard.putNumber("Position", Positions.gridNumber);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;
    double rotationCommanded ;
    
    double xSpeed = -MathUtil.applyDeadband(driver.getLeftY(), OIConstants.kDriveDeadband);
    double ySpeed = -MathUtil.applyDeadband(driver.getLeftX(), OIConstants.kDriveDeadband);
    double rot    = -MathUtil.applyDeadband(driver.getRightX(), OIConstants.kDriveDeadband);
    
    // Drive with pure motions
    int POV = driver.getPOV();
    if (POV >= 0) {
      rot = 0;
      switch ((int)(POV / 45)) {
        case 0: xSpeed =  0.2; ySpeed =  0.0; break;
        case 2: xSpeed =  0.0; ySpeed = -0.2; break;
        case 4: xSpeed = -0.2; ySpeed =  0.0; break;
        case 6: xSpeed =  0.0; ySpeed =  0.2; break;
        default: xSpeed =  0.0; ySpeed =  0.0; break;
      }
    }

    SmartDashboard.putNumber( "Heading", getHeading());
    SmartDashboard.putNumber( "X Move", xSpeed);
    SmartDashboard.putNumber( "Y Move", ySpeed);
    SmartDashboard.putNumber( "Rotate", rot);

    currentHeading = getHeading(); 

    // Rate limit the input commands
    xSpeedCommanded = m_xLimiter.calculate(xSpeed) ;
    ySpeedCommanded = m_yLimiter.calculate(ySpeed);
    rotationCommanded = m_rotLimiter.calculate(rot);
  

      // Check Auto Heading
    if (Math.abs(rot) > 0.02) {
        headingLocked = false;
    } else if (!headingLocked && isNotRotating()) {
        headingLocked = true;
        lockCurrentHeading(); 
    }

    if (headingLocked) {

      rotationCommanded = headingLockController.calculate(currentHeading, headingSetpoint);
        if (Math.abs(rotationCommanded) < 0.1) {
          rotationCommanded = 0;
        } 
    }

    SmartDashboard.putBoolean("Heading Locked", headingLocked);

  
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond * DriveConstants.kSpeedFactor;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond* DriveConstants.kSpeedFactor;
    double rotDelivered = rotationCommanded * DriveConstants.kMaxAngularSpeed * DriveConstants.kTurnFactor;

    move(xSpeedDelivered, ySpeedDelivered, rotDelivered, fieldRelative);

    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putNumber("FCD Heading", getFCDHeading() );
        
  }

  public void newHeadingSetpoint(double newSetpoint) {
    headingSetpoint = newSetpoint;
    headingLockController.reset(currentHeading);
  }

  public void lockCurrentHeading() {
    currentHeading = getHeading(); 
    newHeadingSetpoint(currentHeading);
  }

  public boolean isNotRotating() {
    SmartDashboard.putNumber("Rotate rate", m_gyro.getRate());

    return (Math.abs(m_gyro.getRate()) < AutoConstants.kNotRotating);
  }


  public double resetGyroToZero() {
    m_gyro.reset();
    setFieldOffsets();
    lockCurrentHeading();
    return currentHeading ;
  }

  public void setFieldOffsets() {
    if (DriverStation.getAlliance() == Alliance.Red){
      gyro2FieldOffset = 0.0;
      gyro2FCDOffset = Math.PI;
    } else {
        gyro2FieldOffset = Math.PI;  
        gyro2FCDOffset = Math.PI; 
    }
  }

  public double getHeading() {
      return Math.IEEEremainder(Math.toRadians(-m_gyro.getAngle()) + gyro2FieldOffset, Math.PI * 2);
  }

  public double getFCDHeading() {
      return Math.IEEEremainder(Math.toRadians(-m_gyro.getAngle()) + gyro2FCDOffset, Math.PI * 2);
  }


  public Rotation2d getRotation2d() {
      return Rotation2d.fromRadians(getHeading());
  }

  public Rotation2d getFCDRotation2d() {
      return Rotation2d.fromRadians(getFCDHeading());
  }

  /*
   * implement the desired axis movements
   */
  public void move(double x, double y, double t, boolean fieldRel) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      fieldRel
          ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, t, getFCDRotation2d())
          : new ChassisSpeeds(x, y, t));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

    
  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
