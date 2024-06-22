package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.AutoManager;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import monologue.Logged;
import monologue.Annotations.Log;

/*
This is a class for the swerve drive system on the robot. It utilizes a navX gyro to measure the angle of the robot and a SwerveDriveOdometry to measure the position of the robot. There are four SwerveModule objects, each of which is responsible for the individual swerve module. The class also holds a Field2d object which is used for the robot's position with respect to the field.

The drive() method is used to set the desired speed and angle for the robot. The user can decide if they want the desired rotation and speed to be relative to the field or the robot. The setModuleStates() and setModuleRotation() methods are used to set the desired states of each swerve module. The getPose() method returns the pose of the robot in meters. The resetOdometry() method resets the odometry of the robot to the given pose. The resetToAbsolute() method resets all of the swerve modules to the absolute position. The getStates() and getModulePositions() methods return the current states and positions of each swerve module. The zeroGyro() method sets the yaw of the robot to 0. The getYaw() method returns the yaw of the robot.

In the periodic() method, the robot's odometry is updated, and the yaw of the robot is put to the SmartDashboard. The states and positions of each swerve module is also put to the SmartDashboard.
*/

public class Swerve extends SubsystemBase implements Logged{



  private AHRS gyro;

  //Monolog I Desperately want to love you but you make me do this 😡
  @Log private SwerveModule mod0 = new SwerveModule(0, Constants.Swerve.Mod0.CONSTANTS);
  @Log private SwerveModule mod1 = new SwerveModule(1, Constants.Swerve.Mod1.CONSTANTS);
  @Log private SwerveModule mod2 = new SwerveModule(2, Constants.Swerve.Mod2.CONSTANTS);
  @Log private SwerveModule mod3 = new SwerveModule(3, Constants.Swerve.Mod3.CONSTANTS);
  @Log(key="swerveOffsetCommand") private Command LoggedSwerveOffsetCommand = setAngleOffsetCommand();
  @Log(key="readyForSysIDCommand") private Command loggedReadyForSysIDCommand = positionForSysIDCommand();

  private SwerveDrivePoseEstimator swerveOdometry;
  private SwerveModule[] mSwerveMods;


  SysIdRoutine routine;
  public Swerve(AHRS gyro) {
    this.gyro = gyro;
    zeroGyro();
    
    mSwerveMods = new SwerveModule[] {
        mod0,mod1,mod2,mod3
    };
    
    swerveOdometry = new SwerveDrivePoseEstimator(Constants.Swerve.SWERVE_KINEMATICS, getYaw(),
        getModulePositions(), Constants.Swerve.INITIAL_POSE, Constants.Swerve.STATE_STD_DEVS,
        Constants.VisionConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS);
    
    routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(this::voltageDrive, null, this));

    AutoManager.addSysidCharacterization(
      "Swerve",
      command -> {
        return command
        .beforeStarting(this.positionForSysIDCommand()).andThen(new WaitCommand(1))
        .finallyDo(()-> setAllDriveEnabled(true));
      },
      routine
      );
    AutoManager.addCharacterization("Swerve Offset", setAngleOffsetCommand());
    Preferences.initDouble("offset 0", Constants.Swerve.Mod0.OFFSET_DEGREE);
    Preferences.initDouble("offset 1", Constants.Swerve.Mod1.OFFSET_DEGREE);
    Preferences.initDouble("offset 2", Constants.Swerve.Mod2.OFFSET_DEGREE);
    Preferences.initDouble("offset 3", Constants.Swerve.Mod3.OFFSET_DEGREE);
  }

  public void getPreferences() {
    mSwerveMods[0].setAngleOffset(Preferences.getDouble("offset 0", mSwerveMods[0].getAngleOffset().getDegrees()));
    mSwerveMods[1].setAngleOffset(Preferences.getDouble("offset 1", mSwerveMods[1].getAngleOffset().getDegrees()));
    mSwerveMods[2].setAngleOffset(Preferences.getDouble("offset 2", mSwerveMods[2].getAngleOffset().getDegrees()));
    mSwerveMods[3].setAngleOffset(Preferences.getDouble("offset 3", mSwerveMods[3].getAngleOffset().getDegrees()));
  }
  public SysIdRoutine getSysId() {
    return routine;

  }

  public void setupPathPlanner() {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        AutoConstants.PATH_FOLLOWER_CONFIG,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }
  
  public void voltageDrive(Measure<Voltage> volts) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setVoltage(volts);
    }
  }
  public void setAllDriveEnabled(boolean enabled) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDriveEnabled(enabled);
    }
  }
  public void useVisionMeasurement(Vision.VisionMeasurement measurement) {
    swerveOdometry.addVisionMeasurement(measurement.pose, measurement.timeStamp);
  }

  public Pose2d addVisionMeasurement(Pose2d measurement, double timeStamp) {
    swerveOdometry.addVisionMeasurement(measurement, timeStamp);
    this.log("vision added x", measurement.getX());
    this.log("vision added y", measurement.getY());

    return swerveOdometry.getEstimatedPosition();
  }

  /**
   * Sets the desired speed and angle for the robot. The desired rotation and
   * speed can be relative to the field or the robot.
   * 
   * @param translation   The desired translation.
   * @param rotation      The desired rotation.
   * @param fieldRelative Whether the desired rotation and speed should be
   *                      relative to the field or the robot.
   * @param isOpenLoop    Whether the desired speed should be open loop or closed
   *                      loop.
   */
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop,false);
    }
  }

  public ChassisSpeeds getChassisSpeeds() {
    return Constants.Swerve.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }
  @Log(key = "States")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }

    return states;
  }
  @Log(key = "Desired States")
  public SwerveModuleState[] getModuleDesiredStates() {
    SwerveModuleState[] desiredStates = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      desiredStates[mod.moduleNumber] = mod.getDesiredState();
    }

    return desiredStates;
  }

  public void driveRelative(ChassisSpeeds chassisSpeeds) {
    driveRelative(chassisSpeeds, false);
  }
  public void driveRelative(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop,false);
    }
  }

  public void stop() {
    drive(new Translation2d(0, 0).times(Constants.Swerve.MAX_SPEED),
        0 * Constants.Swerve.MAX_ANGULAR_VELOCITY,
        true, false);
  }

  /**
   * Sets the desired states for each SwerveModule.
   * 
   * @param desiredStates The desired states for each SwerveModule.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates,boolean force) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false,force);
    }
  }

  /**
   * Sets the desired rotation for each SwerveModule.
   * 
   * @param rotation The desired rotation.
   */
  public void setModuleRotation(Rotation2d rotation,boolean force) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(new SwerveModuleState(0, rotation), false,force);
    }
  }

  /* Set individual rotation */
  public void setModule0(Rotation2d rotation) {
    mSwerveMods[0].setDesiredState(new SwerveModuleState(0, rotation), false,false);
  }

  public void setModule1(Rotation2d rotation) {
    mSwerveMods[1].setDesiredState(new SwerveModuleState(0, rotation), false,false);
  }

  public void setModule2(Rotation2d rotation) {
    mSwerveMods[2].setDesiredState(new SwerveModuleState(0, rotation), false,false);
  }

  public void setModule3(Rotation2d rotation) {
    mSwerveMods[3].setDesiredState(new SwerveModuleState(0, rotation), false,false);
  }

  /**
   * Returns the pose of the robot in meters.
   * 
   * @return The pose of the robot in meters.
   */
  @Log(key = "Robot Pose")
  public Pose2d getPose() {
    return swerveOdometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry of the robot to the given pose.
   * 
   * @param pose The pose to reset the odometry to.
   */
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  /**
   * Resets each SwerveModule to the absolute position.
   */
  public void resetToAbsolute() {
    getPreferences();
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  /**
   * Returns the current states of each SwerveModule.
   * 
   * @return The current states of each SwerveModule.
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  /**
   * Returns the current positions of each SwerveModule.
   * 
   * @return The current positions of each SwerveModule.
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }
  /**
   * Returns the current distance of each SwerveModule drive motor.
   * 
   * @return The current distance of each SwerveModule drive motor.
   */
  public double[] getModuleDistances() {
    double[] positions = new double[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition().distanceMeters;
    }
    return positions;
  }


  /**
   * Sets the yaw of the robot to 0.
   */
  public void zeroGyro() {
    if (Robot.isReal()) {
      gyro.zeroYaw();
    } else {
      System.out.println("Zeroing fake gyro");
    }
    // gyro.setYaw(0);
  }

  /**
   * Returns the yaw of the robot.
   * 
   * @return The yaw of the robot.
   */
  public Rotation2d getYaw() {
    if (Robot.isSimulation()) {
      return new Rotation2d(0);
    }
    return gyro.getRotation2d();
  }
  public Command setAngleOffsetCommand() {
    return runOnce(() -> {
      for (SwerveModule mod : mSwerveMods) {
        Preferences.setDouble("offset " + mod.moduleNumber, mod.getCanCoder().getDegrees());
      };
    }).ignoringDisable(true);
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getModulePositions());

    this.log("NavX Yaw", getYaw().getDegrees());
    this.log("NavX Pitch", gyro == null ? 0: gyro.getPitch());

    this.log("NavX Roll", gyro == null ? 0: gyro.getRoll());

    this.log("Acceleration", gyro == null ? 0: gyro.getWorldLinearAccelX());
    this.log("Direction",  gyro == null ? 0:gyro.getCompassHeading());
    for (SwerveModule mod : mSwerveMods) {
      mod.updateCache();
    }
    this.log("Teleop", DriverStation.isTeleopEnabled());
  }

  public void runWheelRadiusCharacterization(double characterizationInput) {
    driveRelative(new ChassisSpeeds(0, 0, characterizationInput),false);
  }
  public double inityaw;
  public Command CheckTimeCommand(boolean isRight) {
    return runOnce(()->inityaw=this.getYaw().getDegrees()).andThen(new WaitCommand(4)).andThen(()->System.out.println((isRight? "right":"left")+","+inityaw+","+Double.toString(this.getYaw().getDegrees()-inityaw)),this);
  }
  public Command positionForSysIDCommand() {
    return this.startEnd(() ->{
          setModuleRotation(new Rotation2d(),true);
          setAllDriveEnabled(false);
        },()-> {}).withTimeout(1);
  }
}