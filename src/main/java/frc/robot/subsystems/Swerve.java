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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;

/*
This is a class for the swerve drive system on the robot. It utilizes a navX gyro to measure the angle of the robot and a SwerveDriveOdometry to measure the position of the robot. There are four SwerveModule objects, each of which is responsible for the individual swerve module. The class also holds a Field2d object which is used for the robot's position with respect to the field.

The drive() method is used to set the desired speed and angle for the robot. The user can decide if they want the desired rotation and speed to be relative to the field or the robot. The setModuleStates() and setModuleRotation() methods are used to set the desired states of each swerve module. The getPose() method returns the pose of the robot in meters. The resetOdometry() method resets the odometry of the robot to the given pose. The resetToAbsolute() method resets all of the swerve modules to the absolute position. The getStates() and getModulePositions() methods return the current states and positions of each swerve module. The zeroGyro() method sets the yaw of the robot to 0. The getYaw() method returns the yaw of the robot.

In the periodic() method, the robot's odometry is updated, and the yaw of the robot is put to the SmartDashboard. The states and positions of each swerve module is also put to the SmartDashboard.
*/

public class Swerve extends SubsystemBase {
  private AHRS gyro;

  private SwerveDrivePoseEstimator swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;
  BuiltInAccelerometer accelerometer;

  SysIdRoutine routine;

  public Swerve(AHRS gyro) {

    accelerometer = new BuiltInAccelerometer();
    this.gyro = gyro;
    // gyro.configFactoryDefault();
    zeroGyro();

    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };
    swerveOdometry = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(),
        getModulePositions(), Constants.Swerve.INITIAL_POSE, Constants.Swerve.stateStdDevs,
        Constants.Swerve.visionStdDevs);
    field = new Field2d();
    SmartDashboard.putData(field);

    
    routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(this::voltageDrive, null, this));

    SmartDashboard.putData("Swerve/SysId/dynamic forward", sysIdDynamic(Direction.kForward));
    SmartDashboard.putData("Swerve/SysId/dynamic backward", sysIdDynamic(Direction.kReverse));
    SmartDashboard.putData("Swerve/SysId/quasistatic forward", sysIdQuasistatic(Direction.kForward));
    SmartDashboard.putData("Swerve/SysId/quasistatic backward", sysIdQuasistatic(Direction.kReverse));
    SmartDashboard.putData("Swerve/subsytem", this);

    Preferences.initDouble("offset 0", Constants.Swerve.Mod0.offsetDegree);
    Preferences.initDouble("offset 1", Constants.Swerve.Mod1.offsetDegree);
    Preferences.initDouble("offset 2", Constants.Swerve.Mod2.offsetDegree);
    Preferences.initDouble("offset 3", Constants.Swerve.Mod3.offsetDegree);

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putData("Swerve/Modules/Mod "+mod.moduleNumber,mod);
    }
  }

  public void getPreferences() {
    mSwerveMods[0].setAngleOffset(Preferences.getDouble("offset 0", mSwerveMods[0].getAngleOffset().getDegrees()));
    mSwerveMods[1].setAngleOffset(Preferences.getDouble("offset 1", mSwerveMods[1].getAngleOffset().getDegrees()));
    mSwerveMods[2].setAngleOffset(Preferences.getDouble("offset 2", mSwerveMods[2].getAngleOffset().getDegrees()));
    mSwerveMods[3].setAngleOffset(Preferences.getDouble("offset 3", mSwerveMods[3].getAngleOffset().getDegrees()));
  }

  public void setupPathPlanner() {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        AutoConstants.pathFollowerConfig,
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

  public Pose2d addVisionMeasurement(Pose2d measurement, double timeStamp) {
    swerveOdometry.addVisionMeasurement(measurement, timeStamp);
    SmartDashboard.putNumber("vision added x", measurement.getX());
    SmartDashboard.putNumber("vision added y", measurement.getY());

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
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public ChassisSpeeds getChassisSpeeds() {
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }

    return states;
  }

  public SwerveModuleState[] getModuleDesiredStates() {
    SwerveModuleState[] desiredStates = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      desiredStates[mod.moduleNumber] = mod.getDesiredState();
    }

    return desiredStates;
  }

  public void driveRelative(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
    }
  }

  public void stop() {
    drive(new Translation2d(0, 0).times(Constants.Swerve.maxSpeed),
        0 * Constants.Swerve.maxAngularVelocity,
        true, false);
  }

  /* Used by SwerveControllerCommand in Auto */
  /**
   * Sets the desired states for each SwerveModule.
   * 
   * @param desiredStates The desired states for each SwerveModule.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  /**
   * Sets the desired rotation for each SwerveModule.
   * 
   * @param rotation The desired rotation.
   */
  public void setModuleRotation(Rotation2d rotation) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(new SwerveModuleState(0, rotation), false);
    }
  }

  /* Set individual rotation */
  public void setModule0(Rotation2d rotation) {
    mSwerveMods[0].setDesiredState(new SwerveModuleState(0, rotation), false);
  }

  public void setModule1(Rotation2d rotation) {
    mSwerveMods[1].setDesiredState(new SwerveModuleState(0, rotation), false);
  }

  public void setModule2(Rotation2d rotation) {
    mSwerveMods[2].setDesiredState(new SwerveModuleState(0, rotation), false);
  }

  public void setModule3(Rotation2d rotation) {
    mSwerveMods[3].setDesiredState(new SwerveModuleState(0, rotation), false);
  }

  /**
   * Returns the pose of the robot in meters.
   * 
   * @return The pose of the robot in meters.
   */
  public Pose2d getPose() {
    return swerveOdometry.getEstimatedPosition();
  }

  /**
   * Returns the Field2d object.
   * 
   * @return The Field2d object.
   */
  public Field2d getField() {
    return field;
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
   * Sets the yaw of the robot to 0.
   */
  public void zeroGyro() {
    gyro.zeroYaw();
    // gyro.setYaw(0);
  }

  /**
   * Returns the yaw of the robot.
   * 
   * @return The yaw of the robot.
   */
  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  StructArrayPublisher<SwerveModuleState> statePublisher = NetworkTableInstance.getDefault()
      .getTable("SmartDashboard/Swerve")
      .getStructArrayTopic("States", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> desiredStatePublisher = NetworkTableInstance.getDefault()
      .getTable("SmartDashboard/Swerve")
      .getStructArrayTopic("Desired States", SwerveModuleState.struct).publish();

  @Override
  public void periodic() {

    statePublisher.set(getModuleStates());
    desiredStatePublisher.set(getModuleDesiredStates());

    swerveOdometry.update(getYaw(), getModulePositions());
    field.setRobotPose(getPose());

    SmartDashboard.putNumber("Pigeon2 Yaw", gyro.getYaw());
    SmartDashboard.putNumber("Pigeon2 Pitch", gyro.getPitch());

    SmartDashboard.putNumber("Pigeon2 Roll", gyro.getRoll());

    SmartDashboard.putNumber("Acceleration", accelerometer.getX());

    for (SwerveModule mod : mSwerveMods) {
      mod.updateCache();
    }
    SmartDashboard.putBoolean("Teleop", DriverStation.isTeleopEnabled());

  }

}
