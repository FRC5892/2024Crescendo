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
    public static final int LEFT_CLIMB_MOTOR_ID = 19;
    public static final int RIGHT_CLIMB_MOTOR_ID = 20;
    public static final double CLIMB_SPEED = 0.8;
    public static final double RETRACT_SPEED = 0.5;
    public static final double LEVEL_SPEED = 0.4;
  }

  public static final class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 13;
    // public static final int DEPLOY_MOTOR_2_ID = 15;
    
    public static final int DEPLOY_MOTOR_ID = 14;

    public static final int BEAM_BREAK_DIO_PORT_ID = 0;
    public static final int DEPLOY_LIMIT_SWITCH_DIO_PORT_ID = 1;
    public static final int RETRACT_LIMIT_SWITCH_DIO_PORT_ID = 2;

    public static final double INTAKE_SPEED = -0.45;
    public static final double OUTTAKE_SPEED = 0.8;
    public static final double OUTTAKE_SPEED_FOR_AMP = 0.55; //0.445;

    public static final double DEPLOY_SPEED = -1;
    public static final double RETRACT_SPEED = 0.75;
    public static final double AMP_RETRACT_SPEED = 1;

    public static final PIDConstants DEPLOY_PID = new PIDConstants(0.02, 0, 0);
    
    public static final double MAX_VELOCITY = 1;
    public static final double MAX_ACCELERATION = 0.5;

    public static final double DEPLOY_ROTATIONS = 0.0;
    public static final double RETRACT_ROTATIONS = 0.65;
    public static final double DEPLOYSLOW_ROTATIONS = 0.15;
    public static final double DEPLOYSLOW_SPEED = -0.3;

  }
  public static final class AmpAssistConstants {
    public static final double RETRACT_POSITION = 0.45;
    public static final double EXTEND_POSITION = 1;
    public static final int LEFT_SERVO_PORT = 0;
    public static final int RIGHT_SERVO_PORT = 1;


  }
  public static final class ShooterConstants {
    public static final int LEFT_KICKER_MOTOR_ID = 16;
    public static final int RIGHT_KICKER_MOTOR_ID = 17;
  
    public static final PIDConstants LEFT_PID = new PIDConstants(0.2, 0, 0);
    public static final PIDConstants RIGHT_PID = new PIDConstants(0.2, 0, 0);
  }

  public static final class VisionConstants {
    public static final String FRONT_CAMERA_NAME = "front";
    public static final String BACK_CAMERA_NAME = "back";

    public static final Transform3d ROBOT_TO_BACK_CAM = new Transform3d(new Translation3d(-0.3302,-0.3302, 0.27305),
        new Rotation3d(0, Units.Degrees.of(-50).in(Units.Radians), 3.14 /*180 deg*/));
    public static final Transform3d ROBOT_TO_FRONT_CAM = new Transform3d(new Translation3d(-0.1524,0, 0.635),
        new Rotation3d(0, Units.Degrees.of(-32).in(Units.Radians), 0));
    public static final String FIELD_LAYOUT_RESOURCE_FILE = AprilTagFields.k2024Crescendo.m_resourceFile;
     /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
    public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
    public static final double NOISY_DISTANCE_METERS = 2.5;
    public static final double AMP_NOISY_DISTANCE_METERS = 1.5;

    public static final double DISTANCE_WEIGHT = 15;
    public static final int TAG_PRESENCE_WEIGHT = 10;

    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
     * meters.
     */
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = VecBuilder
        .fill(
            // if these numbers are less than one, multiplying will do bad things
            1, // x
            1, // y
            Math.PI // theta
        );
  }
  //8.5 off back; 25 off ground; centered;

  public static final class Swerve {
    // TODO: change me
    public static final Pose2d INITIAL_POSE = new Pose2d(0, 0, new Rotation2d(0));

    // Standard deviations of the pose estimate (x position in meters, y position
    // in meters, and heading in radians). Increase these numbers to trust your
    // state estimate less.
    public static final Matrix<N3, N1> STATE_STD_DEVS = VecBuilder.fill(0.05 , 0.05, Units.Degrees.of(2.5).in(Units.Radians));
    public static final double STICK_DEAD_BAND = 0.1;
    public static final boolean INVERT_GYRO = true; // Always ensure Gyro is CCW+ CW-

    /* Autonomous Speeds */

    /* Drivetrain Constants */
    public static final double TRACK_WIDTH = Units.Inches.of(29).in(Units.Meters);
    public static final double WHEEL_BASE = Units.Inches.of(29).in(Units.Meters);
    public static final double WHEEL_DIAMETER = Units.Inches.of(4.0).in(Units.Meters);
    // public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double OPEN_LOOP_RAMP = 2;

    public static final double DRIVE_GEAR_RATIO = (6.75 / 1.0); // 6.75:1 for L2

    public static final double ANGLE_GEAR_RATIO = (12.8 / 1.0); // 12.8:1 for all L's

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    /* Swerve Compensation */
    public static final double VOLTAGE_COMP = 12.0;

    /* Swerve Current Limiting */ 
    public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
    public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 50; //50

    /* TODO: test Angle Motor PID Values (these are default, may tune if needed) */
    public static final double ANGLE_KP = 0.02;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.0;
    public static final double ANGLE_KFF = 0.0;

    /* Drive Motor PID Values */
    public static final double DRIVE_KP = 0.00; // TODO: Tune after characterization
    public static final double DRIVE_KI = 0.0; // leave
    public static final double DRIVE_KD = 0.0; // leave
    public static final double DRIVE_KFF = 0.0; // leave

    /* TODO: characterization drive ff */
    public static final double DRIVE_KS = 0.119;
    public static final double DRIVE_KV = 2.3823;
    public static final double DRIVE_KA = 0.3;

    /* Drive Motor Conversion Factors */
    public static final double DRIVE_CONVERSION_VELOCITY_FACTOR = (((WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO) / 60.0 ) ;
    public static final double ANGLE_CONVERSION_FACTOR = 360.0 / ANGLE_GEAR_RATIO;

    /* TODO: set Swerve Profiling Values  */
    // private static final double SPEED_MULTIPLIER = 0.2;
    public static final double MAX_SPEED = 6.5; // meters per second
    public static final double MAX_ANGULAR_VELOCITY = 5; // TODO: Tune

    /* Neutral Modes */
    public static final IdleMode ANGLE_NEUTRAL_MODE = IdleMode.kBrake;
    public static final IdleMode DRIVE_NEUTRAL_MODE = IdleMode.kBrake;

    // public static final boolean DRIVE_INVERT = true;
    public static final boolean ANGLE_INVERT = false;

    public static final double DRIVE_FUDGE_FACTOR = 0.060509807 * 0.7;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int DRIVE_MOTOR_ID = 1;
      public static final int ANGLE_MOTOR_ID = 2;
      public static final int CAN_CODER_ID = 9;

      public static final boolean DRIVE_INVERT = false;
      public static final double OFFSET_DEGREE = 331.69;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(OFFSET_DEGREE);
      public static final Rotation2d BALANCE_OFFSET = Rotation2d.fromDegrees(OFFSET_DEGREE + 45);

      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_INVERT, DRIVE_MOTOR_ID,
          ANGLE_MOTOR_ID,
          CAN_CODER_ID, ANGLE_OFFSET);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final boolean DRIVE_INVERT = false;
      public static final int DRIVE_MOTOR_ID = 3;
      public static final int ANGLE_MOTOR_ID = 4;
      public static final int CAN_CODER_ID = 10;

      public static final double OFFSET_DEGREE = -103.5;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(OFFSET_DEGREE);
      public static final Rotation2d BALANCE_OFFSET = Rotation2d.fromDegrees(OFFSET_DEGREE - 45);

      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_INVERT, DRIVE_MOTOR_ID,
          ANGLE_MOTOR_ID,
          CAN_CODER_ID, ANGLE_OFFSET);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final boolean DRIVE_INVERT = true;
      public static final int DRIVE_MOTOR_ID = 7;
      public static final int ANGLE_MOTOR_ID = 8;
      public static final int CAN_CODER_ID = 12;

      public static final double OFFSET_DEGREE = -161.89;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(OFFSET_DEGREE);
      public static final Rotation2d BALANCE_OFFSET = Rotation2d.fromDegrees(OFFSET_DEGREE + 45);

      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_INVERT, DRIVE_MOTOR_ID,
          ANGLE_MOTOR_ID,
          CAN_CODER_ID, ANGLE_OFFSET);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final boolean DRIVE_INVERT = true;
      public static final int DRIVE_MOTOR_ID = 5;
      public static final int ANGLE_MOTOR_ID = 6;
      public static final int CAN_CODER_ID = 11;

      public static final double OFFSET_DEGREE = 58.26;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(OFFSET_DEGREE);
      public static final Rotation2d BALANCE_OFFSET = Rotation2d.fromDegrees(OFFSET_DEGREE - 45);

      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_INVERT, DRIVE_MOTOR_ID,
        ANGLE_MOTOR_ID,
        CAN_CODER_ID, ANGLE_OFFSET);
    }
  }

  public static final class AutoConstants {
    public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
    new PIDConstants(0.2, 0.0, 0.0), // Translation PID constants
    new PIDConstants(1, 0.0, 0.0), // Rotation PID constants
    6.5, // Max module speed, in m/s
    Swerve.WHEEL_BASE, // Drive base radius in meters. Distance from robot center to furthest module.
    new ReplanningConfig(true, true)); // Default path replanning config. See the API for the options here
  }

  /* LED Ports */
  public static final class LEDConstants {
    public static final int LED_PORT = 0;
    public static final int LED_LENGTH = 0;
  }
}