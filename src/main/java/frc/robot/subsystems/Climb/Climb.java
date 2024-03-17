// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  private CANSparkMax leftClimb;
  private CANSparkMax rightClimb;
  private RelativeEncoder leftClimbEncoder;

  /** Creates a new Climb. */
  public Climb() {
    leftClimb = new CANSparkMax(ClimbConstants.LEFT_CLIMB_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    rightClimb = new CANSparkMax(ClimbConstants.RIGHT_CLIMB_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

    leftClimbEncoder = leftClimb.getEncoder();
    leftClimbEncoder.setPositionConversionFactor(0.50);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climbRightMotor(double speed) {
    rightClimb.set(Constants.ClimbConstants.CLIMB_SPEED);
  }

  public void climbLeftMotor(double speed) {
    leftClimb.set(Constants.ClimbConstants.CLIMB_SPEED);
  }

  public void retractRightMotor(double speed) {
    rightClimb.set(-Constants.ClimbConstants.RETRACT_SPEED);
  }

  public void retractLeftMotor(double speed) {
    leftClimb.set(-Constants.ClimbConstants.RETRACT_SPEED);
  }

  public void stopLeft() {
    leftClimb.set(0);
  }

  public void stopRight() {
    rightClimb.set(0);
  }

  public void climbMotors() {
    climbLeftMotor(Constants.ClimbConstants.CLIMB_SPEED);
    climbRightMotor(Constants.ClimbConstants.CLIMB_SPEED);
  }
  
  public void retractMotors() {
    retractLeftMotor(Constants.ClimbConstants.RETRACT_SPEED);
    retractRightMotor(Constants.ClimbConstants.RETRACT_SPEED);
  }

  public void stopMotors() {
    stopLeft();
    stopRight();
  }

  /* Commands */
  public Command climbUp() {
    return runEnd(this::climbMotors, this::stopMotors);
  }

  public Command climbDown() {
    return runEnd(this::retractMotors, this::stopMotors);
  }

  public Command tiltLeft() {
    return runEnd(() -> this.retractLeftMotor(Constants.ClimbConstants.LEVEL_SPEED), this::stopLeft);
  }

  public Command tiltRight() {
    return runEnd(() -> this.retractRightMotor(Constants.ClimbConstants.LEVEL_SPEED), this::stopRight);
  }
}