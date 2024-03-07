package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

  public static final class ClimbConstants {
    public static final int leftClimbMotorID = 19;
    public static final int rightClimbMotorID = 20;
    public static final double climbSpeed = 0.8;
    public static final double retractSpeed = 0.5;
    public static final double levelSpeed = 0.4;
  }

  public static final class IntakeConstants {
    public static final int intakeMotorID = 13;
    public static final int deployMotorID = 14;

    public static final int beamBreakDIOPortID = 0;
    public static final int deployLimitSwitchDIOPortID = 1;
    public static final int retractLimitSwitchDIOPortID = 2;

    public static final double intakeSpeed = -0.8;
    public static final double outtakeSpeed = 0.5;
    public static final double deploySpeed = -0.5;
    public static final double retractSpeed = 0.5;
    public static final double ampRetractSpeed = 1;

    public static final PIDConstants deployPID = new PIDConstants(0.02, 0, 0);
    
    public static final double maxVelocity = 1;
    public static final double maxAcceleration = 0.5;

    public static final double deployRotations = 0.0;
    public static final double retractRotations = 0.65;



  }

  public static final class ShooterConstants {
    public static final int leftKickerMotorID = 15;
    public static final int rightKickerMotorID = 16;
    
    public static final PIDConstants leftPID = new PIDConstants(0.2, 0, 0);
    public static final PIDConstants rightPID = new PIDConstants(0.2, 0, 0);

  }

  public static final class VisionConstants {
    public static final String cameraName = "defaultCamera";
    public static final Transform3d robotToCam = new Transform3d(new Translation3d(-0.3302,-0.3302, 0.27305),
        new Rotation3d(0, Units.Degrees.of(-50).in(Units.Radians), 3.14 /*180 deg*/));
    public static final String fieldLayoutResourceFile = AprilTagFields.k2024Crescendo.m_resourceFile;
  }

  public static final class Swerve {
    // TODO: change me
    public static final Pose2d INITIAL_POSE = new Pose2d(0, 0, new Rotation2d(0));

    // Standard deviations of the pose estimate (x position in meters, y position
    // in meters, and heading in radians). Increase these numbers to trust your
    // state estimate
    // less.
    public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(10, 10, Units.Degrees.of(5).in(Units.Radians));

    // Standard deviations of the vision pose measurement (x position
    // in meters, y position in meters, and heading in radians). Increase these
    // numbers to trust
    // the vision pose measurement less.
    public static final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.5, 0.5, Units.Degrees.of(30).in(Units.Radians));

    public static final double stickDeadBand = 0.1;
    public static final int pigeonID = 13;
    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    /* Autonomous Speeds */

    /* Drivetrain Constants */
    public static final double trackWidth = Units.Inches.of(29).in(Units.Meters);
    public static final double wheelBase = Units.Inches.of(29).in(Units.Meters);
    public static final double wheelDiameter = Units.Inches.of(4.0).in(Units.Meters);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 2;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1 for L2

    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1 for all L's

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 50;

    /* TODO: test Angle Motor PID Values (these are default, may tune if needed) */
    public static final double angleKP = 0.02;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.00; // TODO: Tune after characterization
    public static final double driveKI = 0.0; // leave
    public static final double driveKD = 0.0; // leave
    public static final double driveKFF = 0.0; // leave

    /* TODO: characterization drive ff */
    public static final double driveKS = 0.119;
    public static final double driveKV = 2.3823;
    public static final double driveKA = 0.3;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionVelocityFactor = ((wheelDiameter * Math.PI) / driveGearRatio) / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* TODO: set Swerve Profiling Values */
    // private static final double SPEED_MULTIPLIER = 0.2;
    public static final double maxSpeed = 5.5; // meters per second
    public static final double maxAngularVelocity = 5; // TODO: Tune

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    // public static final boolean driveInvert = true;
    public static final boolean angleInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 9;

      public static final boolean driveInvert = false;
      public static final double offsetDegree = 331.69;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(offsetDegree);
      public static final Rotation2d balanceOffset = Rotation2d.fromDegrees(offsetDegree + 45);
      public static final double conversionFactor = 0.060509807;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveInvert, driveMotorID,
          angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final boolean driveInvert = false;
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 10;

      public static final double offsetDegree = -103.5;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(offsetDegree);
      public static final Rotation2d balanceOffset = Rotation2d.fromDegrees(offsetDegree - 45);
      public static final double conversionFactor = 0.060509807;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveInvert, driveMotorID,
          angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final boolean driveInvert = true;
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 12;

      public static final double offsetDegree = -161.89;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(offsetDegree);
      public static final Rotation2d balanceOffset = Rotation2d.fromDegrees(offsetDegree + 45);
      public static final double conversionFactor = 0.060509807;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveInvert, driveMotorID,
          angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final boolean driveInvert = true;
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 11;

      public static final double offsetDegree = 58.26;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(offsetDegree);
      public static final Rotation2d balanceOffset = Rotation2d.fromDegrees(offsetDegree - 45);
      public static final double conversionFactor = 0.060509807;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveInvert, driveMotorID,
          angleMotorID,
          canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final HolonomicPathFollowerConfig pathFollowerConfig= new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
    //TODO: check if this works
    // we changed replanning config and pid constants.

    //5 is the default
    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
    Swerve.maxSpeed, // Max module speed, in m/s
    Swerve.wheelBase, // Drive base radius in meters. Distance from robot center to furthest module.
    new ReplanningConfig(true,true)); // Default path replanning config. See the API for the options here
  }

  /* LED Ports */
  public static final class LEDConstants {
    public static final int ledPort = 0;
    public static final int ledLength = 105;
  }
}
