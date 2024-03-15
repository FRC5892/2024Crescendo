package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;


public class SwerveModule implements Sendable {
  public int moduleNumber;

  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder;

  private final SparkPIDController driveController;
  private final SparkPIDController angleController; 

  private boolean isDriveEnabled = true;
  private boolean isAngleEnabled = true;

  private double cachedCanCoderPosition =0;
  private double cachedModPosition = 0;
  private SwerveModuleState cachedState = new SwerveModuleState();


  private SwerveModuleState desiredState = new SwerveModuleState(0, new Rotation2d(0));

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    driveMotor.setInverted(moduleConstants.driverInvert);

    // driveEncoder.setPositionConversionFactor(moduleConstants.conversionFactor)
    configDriveMotor();

    lastAngle = getState().angle;

  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    this.desiredState = desiredState;
    // Custom optimize command, since default WPILib optimize assumes
    // continuous controller which REV and CTRE are not


    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }


  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (!isDriveEnabled) {
      driveMotor.set(0);
      return;
    }
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }
  public void setVoltage(Measure<Voltage> volts) {
    driveMotor.setVoltage(volts.in(Units.Volts));
  }

  private void setAngle(SwerveModuleState desiredState) {
    if (!isAngleEnabled) {
      angleMotor.set(0);
      return;
    }
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  public void resetToAbsolute() {
    // System.out.println("resetAbsolute \n \n \n \n reset to absolute");
    
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  private void configAngleEncoder() {
    //angleEncoder.configFactoryDefault();
    // CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.Swerve.angleInvert);
    angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
    angleController.setPositionPIDWrappingEnabled(true);
    angleController.setPositionPIDWrappingMinInput(-180.0);
    angleController.setPositionPIDWrappingMaxInput(180.0);
    angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    burnAngleFlash(1);

    // TODO: fix me
    // try {
    // Thread.sleep(200l);
    // } catch (InterruptedException e) {
    //   throw new RuntimeException();
    // }
    // angleMotor.burnFlash();

    
    resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kVelocityOnly);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    driveEncoder.setPositionConversionFactor(0.060509807);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    // driveController.setP(Constants.Swerve.angleKP);
    // driveController.setI(Constants.Swerve.angleKI);
    // driveController.setD(Constants.Swerve.angleKD);
    // driveController.setFF(Constants.Swerve.angleKFF);
    driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    driveEncoder.setPosition(0.0);
    burnDriveFlash(1);

    // driveMotor.setInverted(Constants.Swerve.driveInvert);
  }

  private boolean burnDriveFlash(int attempt) {
    driveController.setP(Constants.Swerve.driveKP);
    driveController.setI(Constants.Swerve.driveKI);
    driveController.setD(Constants.Swerve.driveKD);
    driveController.setFF(Constants.Swerve.driveKFF);
    try {
      Thread.sleep(300);
      driveMotor.burnFlash();
      Thread.sleep(300);
    } catch (Exception e) {
      return false;
    }
    if (
          driveController.getP()==Constants.Swerve.driveKP
        &&driveController.getI()==Constants.Swerve.driveKI
        &&driveController.getD()==Constants.Swerve.driveKD
        &&driveController.getFF()==Constants.Swerve.driveKFF
        ) {
      return true;
    } else {  
      if (attempt >= 5) return false;
      return burnDriveFlash(attempt+1);
    }
  }
  private boolean burnAngleFlash(int attempt) {
    angleController.setP(Constants.Swerve.angleKP);
    angleController.setI(Constants.Swerve.angleKI);
    angleController.setD(Constants.Swerve.angleKD);
    angleController.setFF(Constants.Swerve.angleKFF);
    try {
      Thread.sleep(300);
      driveMotor.burnFlash();
      Thread.sleep(300);
    } catch (Exception e) {
      return false;
    }
    if (
          angleController.getP()==Constants.Swerve.angleKP
        &&angleController.getI()==Constants.Swerve.angleKI
        &&angleController.getD()==Constants.Swerve.angleKD
        &&angleController.getFF()==Constants.Swerve.angleKFF
        ) {
      return true;
    } else {  
      if (attempt >= 5) return false;
      return burnAngleFlash(attempt+1);
    }
  }

  public SwerveModuleState getDesiredState() {
    return desiredState;
  }
  public void setAngleOffset(double degree) {
    angleOffset = new Rotation2d(Math.toRadians(degree));
  }
  public Rotation2d getAngleOffset() {
    return angleOffset;
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    //return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }
  public double getAbsolutePosition() {
    return angleEncoder.getAbsolutePosition().getValue();
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public SwerveModuleState getState() {
    double velocity = driveEncoder.getVelocity();
    return new SwerveModuleState(velocity, getAngle());
  }

  public SwerveModulePosition getPosition() {
    double position = driveEncoder.getPosition();
    return new SwerveModulePosition(position, getAngle());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Angle Enabled", () -> isAngleEnabled, this::setAngleEnabled);
    builder.addBooleanProperty("Drive Enabled", () -> isDriveEnabled, this::setDriveEnabled);
    builder.addDoubleProperty("Stats/Cancoder", ()->cachedCanCoderPosition, null);
    builder.addDoubleProperty("Stats/Integrated", ()->cachedState.angle.getDegrees(), null);
    builder.addDoubleProperty("Stats/Velocity", ()->cachedState.speedMetersPerSecond, null);
    builder.addDoubleProperty("Stats/Position", ()->cachedModPosition, null);
    builder.addDoubleProperty("Stats/Setpoint Angle", ()->desiredState.angle.getDegrees(), null);
    builder.addDoubleProperty("Stats/Setpoint Velocity", ()->desiredState.speedMetersPerSecond, null);
  }
  public void updateCache() {
    cachedCanCoderPosition = getCanCoder().getDegrees();
    cachedModPosition = this.getPosition().distanceMeters;
    cachedState = getState();
  }
  public void setAngleEnabled(boolean enabled) {
    if (isAngleEnabled!=enabled) {
        angleMotor.setIdleMode(enabled?IdleMode.kBrake:IdleMode.kCoast);
    }
    isAngleEnabled = enabled;
  }
  public void setDriveEnabled(boolean enabled) {
    if (isDriveEnabled!=enabled) {
        driveMotor.setIdleMode(enabled?IdleMode.kBrake:IdleMode.kCoast);
    }
    isDriveEnabled = enabled;
  }


}
