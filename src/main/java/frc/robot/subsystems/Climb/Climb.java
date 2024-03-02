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
    leftClimb = new CANSparkMax(ClimbConstants.leftClimbMotorID, CANSparkLowLevel.MotorType.kBrushless);
    rightClimb = new CANSparkMax(ClimbConstants.rightClimbMotorID, CANSparkLowLevel.MotorType.kBrushless);

    leftClimbEncoder = leftClimb.getEncoder();
    leftClimbEncoder.setPositionConversionFactor(0.50);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climbRightMotor() {
    rightClimb.set(Constants.ClimbConstants.climbSpeed);
  }

  public void climbLeftMotor() {
    leftClimb.set(Constants.ClimbConstants.climbSpeed);
  }

  public void retractRightMotor() {
    rightClimb.set(-Constants.ClimbConstants.climbSpeed);
  }

  public void retractLeftMotor() {
    leftClimb.set(-Constants.ClimbConstants.climbSpeed);
  }

  public void stopLeft() {
    leftClimb.set(0);
  }

  public void stopRight() {
    rightClimb.set(0);
  }

  public void climbMotors() {
    climbLeftMotor();
    climbRightMotor();
  }
  
  public void retractMotors() {
    retractLeftMotor();
    retractRightMotor();
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

  public Command climbLeftDown() {
    return runEnd(this::climbLeftMotor, this::stopLeft);
  }

  public Command tiltLeft() {
    return runEnd(this::retractLeftMotor, this::stopLeft);
  }

  public Command tiltRight() {
    return runEnd(this::retractRightMotor, this::stopRight);
  }
}