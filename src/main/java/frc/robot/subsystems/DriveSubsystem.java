// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.PhotonCameraWrapper;
import frc.robot.Shared;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GPMConstants;
import frc.robot.Constants.NavConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP); 
  private double gyro2FieldOffset = 0;

  private final PhotonCameraWrapper pcw = new PhotonCameraWrapper();

  private SlewRateLimiter m_xLimiter = new SlewRateLimiter(DriveConstants.kLinearSlewRate);
  private SlewRateLimiter m_yLimiter = new SlewRateLimiter(DriveConstants.kLinearSlewRate);

  private SlewRateLimiter m_xArmOutLimiter = new SlewRateLimiter(DriveConstants.kLinearSlewArmRate);
  private SlewRateLimiter m_yArmOutLimiter = new SlewRateLimiter(DriveConstants.kLinearSlewArmRate);

  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private SlewRateLimiter m_rotArmOutLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewArmRate);
  
  private PS4Controller driver;
  private Joystick copilot_1;
  private Joystick copilot_2;
  private ProfiledPIDController headingLockController;
  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  
  private boolean headingLocked = false;
  private double  currentHeading = 0;
  private double  headingSetpoint = 0;  
  private boolean disableZoom = false;
  private boolean dangerZone = false;

  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }, new Pose2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(PS4Controller driver, Joystick copilot_1, Joystick copilot_2) {
    this.driver = driver;
    this.copilot_1 = copilot_1;
    this.copilot_2 = copilot_2;
    this.dangerZone = false;

    headingLockController = new ProfiledPIDController(AutoConstants.kPHeadingLockController, 0, 0, AutoConstants.kHeadingLockConstraints );
    headingLockController.enableContinuousInput(-Math.PI, Math.PI);
    xController = new ProfiledPIDController(AutoConstants.kPXController, 0, 0, AutoConstants.kTranslateControllerConstraints);
    yController = new ProfiledPIDController(AutoConstants.kPYController, 0, 0, AutoConstants.kTranslateControllerConstraints);
      
  }

  // ===   Commands that call driveSystem methods  =======
  public CommandBase setXCmd() {return this.runOnce(() -> setX());}
  public CommandBase setStaightCmd() {return this.runOnce(() -> setStraight());}
  public CommandBase moveCmd(double x, double y, double t, boolean fieldRel) {return this.runOnce(() -> move(x, y, t, fieldRel));}
  public CommandBase stopCmd() {return this.runOnce(() -> stop());}
  public CommandBase useAprilTagsCmd(boolean useTags) {return this.runOnce(() -> useAprilTags(useTags));}
  public CommandBase lockCurrentHeadingCmd() {return this.runOnce(() -> lockCurrentHeading());}
  public CommandBase newHeadingSetpointCmd(double newSetpoint) {return this.runOnce(() -> newHeadingSetpoint(newSetpoint));}

 
  public void init() {
    setFieldOffsets();
    lockCurrentHeading();
    resetOdometry(Shared.currentPose);
    useAprilTags(true);
    for (int i=1; i < 10; i++) {
      driver.getRawButtonPressed(i);
    }
    disableZoom = SmartDashboard.putBoolean("Disable Zoom", false);

    // square up the robot by setting the heading to 0 or 180
    squareUp();
  }

  @Override
  public void periodic() {

    if(driver.getRawButtonPressed(OIConstants.kDriverGyroReset)){
      resetGyroToZero();
      lockCurrentHeading();
      // resetOdometry(new Pose2d());  // Temporary
    }
   
    // Update the odometry in the periodic block
    m_odometry.update(
      Rotation2d.fromRadians(getHeading()),
      new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

      if (Shared.useAprilTags) {
        Optional<EstimatedRobotPose> result = pcw.getEstimatedGlobalPose(m_odometry.getEstimatedPosition()); 

        Shared.canSeeAprilTag = result.isPresent();
        if (Shared.canSeeAprilTag) {
          EstimatedRobotPose camPose = result.get();
          Pose2d camPose2d = camPose.estimatedPose.toPose2d();
          Pose2d gyroCamPose2d = new Pose2d(camPose2d.getTranslation(), getRotation2d());  // use gyro heading
          
          m_odometry.addVisionMeasurement(gyroCamPose2d, camPose.timestampSeconds);
      }
    }
     
    // Share current position
    Shared.currentPose = getPose();
        
    SmartDashboard.putString("Est Pose", getPose().toString());
    SmartDashboard.putNumber("Field Y", getPose().getY());
    SmartDashboard.putNumber("Heading Deg", Math.toDegrees(getHeading()));
    SmartDashboard.putNumber("Level", Shared.gridLevel);
    SmartDashboard.putNumber("Position", Shared.gridNumber);
    SmartDashboard.putNumber( "Pitch", getPitch());
   
    disableZoom = SmartDashboard.getBoolean("Disable Zoom", false);
    SmartDashboard.putBoolean("Zoom Disabled", disableZoom);   
     SmartDashboard.putBoolean("Defense Mode", Shared.defenseModeOn);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
      getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info. ================================================
   *
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   */
  public void drive() {

    double xSpeedLimited;
    double ySpeedLimited;
    double turnSpeedLimited ;
    double xSpeed    ;
    double ySpeed    ;
    double turnSpeed ;
    
    getHeading(); 
    
    // if Left Bumper being pressed, apply strong deadband to encourage orthogonal movement.
    if (driver.getL1Button()) {
      xSpeed     = -MathUtil.applyDeadband(driver.getLeftY(), OIConstants.kDriveReallyDeadband) *  DriveConstants.kSpeedFactor;
      ySpeed     = -MathUtil.applyDeadband(driver.getLeftX(), OIConstants.kDriveReallyDeadband) * DriveConstants.kSpeedFactor;
      turnSpeed  = -MathUtil.applyDeadband(driver.getRightX(), OIConstants.kDriveReallyDeadband) * DriveConstants.kTurnFactor;
    } else {
      xSpeed     = -MathUtil.applyDeadband(driver.getLeftY(), OIConstants.kDriveDeadband) *  DriveConstants.kSpeedFactor;
      ySpeed     = -MathUtil.applyDeadband(driver.getLeftX(), OIConstants.kDriveDeadband) * DriveConstants.kSpeedFactor;
      turnSpeed  = -MathUtil.applyDeadband(driver.getRightX(), OIConstants.kDriveDeadband) * DriveConstants.kTurnFactor;
    }
    
    dangerZone = false;

    // reduce speed if we are in Power cable danger zone
    if ((Shared.currentPose.getY() < 2.0) && (
         (((DriverStation.getAlliance() == Alliance.Red)  && (Shared.currentPose.getX() > 10.0)) || 
          ((DriverStation.getAlliance() == Alliance.Blue) && (Shared.currentPose.getX() < 6.5 ))   )
                                              )) {
      dangerZone = true;
    }

    SmartDashboard.putBoolean("Danger Zone", dangerZone);

    // Limit speed if we are in danger zone
    if (dangerZone) {
      xSpeed = Math.min(xSpeed, DriveConstants.kSafeSpeedFactor);
      ySpeed = Math.min(ySpeed, DriveConstants.kSafeSpeedFactor);
    }

    // Allow driver to square up heading
    if(driver.getRawButtonPressed(OIConstants.kDriverSquareUp)){
      squareUp();  //  point to 0 or 180.  Whichever is closest
    }

    // Drive with pure motions
    int POV = driver.getPOV();
    if (POV >= 0) {
      turnSpeed = 0;
      switch ((int)(POV / 45)) {
        case 0:  xSpeed =  DriveConstants.kDPADSpeed; ySpeed =  0.0; break;
        case 2:  xSpeed =  0.0;                       ySpeed = -DriveConstants.kDPADSpeed; break;
        case 4:  xSpeed = -DriveConstants.kDPADSpeed; ySpeed =  0.0; break;
        case 6:  xSpeed =  0.0;                       ySpeed =  DriveConstants.kDPADSpeed; break;
        default: xSpeed =  0.0;                       ySpeed =  0.0; break;
      }
    } else if (driver.getCircleButton()) {
      // determine desired approach speed
      xSpeed = getHumanApproachRate();
    }

    // look to flip direction
    if (driver.getR1ButtonPressed() && (Shared.armPosition < GPMConstants.kArmSafeToSpinn)) {
      if (Math.abs(currentHeading) < Math.PI/2) {
        newHeadingSetpoint(Math.PI);
      } else {
        newHeadingSetpoint(0);
      }
    }

    // =========================================================================================================
    // Rate limit the input commands.  
    // 1) No limit if we are doing defense.  
    // 2) Moderate rate limit if the arm is in.
    // 3) Max rate limit if the arm is out.
    Shared.defenseModeOn = copilot_2.getRawButton(OIConstants.kCP2Pos1);
    if(Shared.defenseModeOn) {

      // Make the spin follow the lateral
      //turnSpeed = ySpeed;

      // If we are heading away from driver station to pick up scoring element, 
      // and we are in Defense mode, then reverse the sping direction to avoid getting stuck on defender.
      //if (!Shared.haveScoringElement) {
      //  turnSpeed *= -1.0;
      //}

      xSpeedLimited = xSpeed ;
      ySpeedLimited = ySpeed;
      turnSpeedLimited = turnSpeed;

      // Tell the other limiters what is going on.
      m_xLimiter.reset(xSpeed);
      m_yLimiter.reset(ySpeed);
      m_rotLimiter.reset(turnSpeed);

      m_xArmOutLimiter.reset(xSpeed);
      m_yArmOutLimiter.reset(ySpeed);
      m_rotArmOutLimiter.reset(turnSpeed);

    } else if(Shared.armPosition < GPMConstants.kArmSafeToSpinn){
      xSpeedLimited = m_xLimiter.calculate(xSpeed) ;
      ySpeedLimited = m_yLimiter.calculate(ySpeed);
      turnSpeedLimited = m_rotLimiter.calculate(turnSpeed);

      // Tell the other limiters what is going on.
      m_xArmOutLimiter.reset(xSpeed);
      m_yArmOutLimiter.reset(ySpeed);
      m_rotArmOutLimiter.reset(turnSpeed);

    } else {
      xSpeedLimited = m_xArmOutLimiter.calculate(xSpeed);
      ySpeedLimited = m_yArmOutLimiter.calculate(ySpeed);
      turnSpeedLimited = m_rotArmOutLimiter.calculate(turnSpeed);

      // Tell the other limiters what is going on.
      m_xLimiter.reset(xSpeed);
      m_yLimiter.reset(ySpeed);
      m_rotLimiter.reset(turnSpeed);
    }
    
    // ========================================================================================
      
    // Check Auto Heading
    if (Math.abs(turnSpeed) > 0.02) {
      headingLocked = false;
    } else if (!headingLocked && isNotRotating()) {
      headingLocked = true;
      lockCurrentHeading();
    } 

    if (headingLocked) {
      turnSpeedLimited = headingLockController.calculate(currentHeading, headingSetpoint);
      if (Math.abs(turnSpeedLimited) < 0.025) {
        turnSpeedLimited = 0;
      } 
    }
  
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedMPS    = xSpeedLimited    * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedMPS    = ySpeedLimited    * DriveConstants.kMaxSpeedMetersPerSecond;
    double turnSpeedRPS = turnSpeedLimited * DriveConstants.kMaxAngularSpeed;

    // Drive based on current goals.
    if (driver.getCircleButton()) {
      move(xSpeedMPS, ySpeedMPS, turnSpeedRPS, false); // Drive Forward to collect game piece
    } else if(driver.getL2Button()){
      setX(); 
    } else {
      move(xSpeedMPS, ySpeedMPS, turnSpeedRPS, true);  // Drive based on field centric commands
    }

    SmartDashboard.putNumber( "X Move", xSpeed);
    SmartDashboard.putNumber( "Y Move", ySpeed);
    SmartDashboard.putNumber( "Rotate", turnSpeed);
    SmartDashboard.putBoolean( "Heading Locked", headingLocked);
    SmartDashboard.putString( "Target", Shared.targetPose.toString());
  }
  
  /***
   * Determines the fastest way to get to human player station pickup point.
   * Use the current robot pose and compares wth the desired X value of the red/blue feeder station.
   * Maxes out at the current DPAD movement rate.
   * 
   * @return  Variable Joystick rate for setting forward speed.
   */
  private double getHumanApproachRate() {
    double xError = 0;
    double rate = DriveConstants.kMinApproachRate;

    // Determine how far out from the min range we are.
    // only use this data to boost speed if we are pointing in the correct direction 
    if(!disableZoom && copilot_1.getRawButton(OIConstants.kCP1ExtendFeeder)){
      if (DriverStation.getAlliance() == Alliance.Red) {
        if (Math.abs(MathUtil.angleModulus(NavConstants.redFeederDir - getHeading())) < (Math.PI / 4)) {
          xError = Shared.currentPose.getX() - NavConstants.redFeederX ;
        }
      } else {
        if (Math.abs(MathUtil.angleModulus(NavConstants.blueFeederDir - getHeading())) < (Math.PI / 4)) {
          xError = NavConstants.blueFeederX  - Shared.currentPose.getX() ;
        }
      }
    }
    
    xError = Math.min(xError, 1.0);
    if (xError > 0) {
      rate += xError * DriveConstants.kDPADSpeed;
    }

    SmartDashboard.putNumber("Human Distance", xError);
    SmartDashboard.putNumber("Human Angle", Math.abs(MathUtil.angleModulus(NavConstants.blueFeederDir - getHeading())));
    SmartDashboard.putNumber("Human Rate",     rate);
    return rate;
  }

  public void initDrivePIDs() {
    newHeadingSetpoint(Shared.targetPose.getRotation().getRadians());
    xController.reset(getPose().getX());
    yController.reset(getPose().getY());
  }

  public void driveToLocation() {
    double xSpeedDelivered = -xController.calculate(getPose().getX(), Shared.targetPose.getX());
    double ySpeedDelivered = -yController.calculate(getPose().getY(), Shared.targetPose.getY());
    double rotDelivered    = headingLockController.calculate(headingLockController.calculate(currentHeading, headingSetpoint)) / 20;
    move(xSpeedDelivered, ySpeedDelivered, rotDelivered, true);  // Drive based on field centric commands
  }

  public void newHeadingSetpoint(double newSetpoint) {
    headingSetpoint = newSetpoint;
    headingLockController.reset(currentHeading);
  }

  public void lockCurrentHeading() {
    newHeadingSetpoint(getHeading());
  }

  public void squareUp() {
    if (Math.abs(getHeading()) < Math.PI / 2) {
      newHeadingSetpoint(0);
    } else {
      newHeadingSetpoint(Math.PI);
    }
  }

  public void useAprilTags(boolean useTags) {
    Shared.useAprilTags = useTags;
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
    } else {
      gyro2FieldOffset = Math.PI;  
    }
  }

  /***
   * Reads heading from gyro, adjusts for field orientation and sets current Heading member.
   * @return
   */
  public double getHeading() {
    currentHeading = Math.IEEEremainder(Math.toRadians(-m_gyro.getAngle()) + gyro2FieldOffset, Math.PI * 2);
      return currentHeading;
  }

  public double getPitch() {
    return -m_gyro.getRoll();
  }

  public double getRoll() {
    return -m_gyro.getPitch();
  }
  
  public double getFCDHeading() {
      return Math.IEEEremainder(Math.toRadians(-m_gyro.getAngle()) + Math.PI, Math.PI * 2);
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
 
  public void stop() {
    move(0,0,0, false);
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
   * Sets the wheels into an || formation to Start staight out.
   */
  public void setStraight() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
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
