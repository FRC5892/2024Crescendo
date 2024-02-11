// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  CANSparkMax leftClimb;
  CANSparkMax rightClimb;

  /** Creates a new Climb. */
  public Climb() {
    leftClimb = new CANSparkMax(Constants.ClimbConstants.leftClimbMotorID, CANSparkLowLevel.MotorType.kBrushless);
    rightClimb = new CANSparkMax(Constants.ClimbConstants.rightClimbMotorID, CANSparkLowLevel.MotorType.kBrushless);

    rightClimb.setIdleMode(IdleMode.kBrake);
    leftClimb.setIdleMode(IdleMode.kBrake);
    rightClimb.follow(leftClimb);
    
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void climbMotorsForward() {
    leftClimb.set(Constants.ClimbConstants.climbSpeed);
  }
    
  public void climbMotorsReverse() {
    leftClimb.set(-Constants.ClimbConstants.climbSpeed);
  }

  public void climbMotorsStop() {
    leftClimb.set(0);
  }

  public Command climbUp() {
    return runEnd(this::climbMotorsForward, this::climbMotorsStop);
  }

  public Command climbDown() {
    return runEnd(this::climbMotorsReverse, this::climbMotorsStop);
  }

}