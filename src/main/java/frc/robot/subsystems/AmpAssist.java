// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AmpAssistConstants;

public class AmpAssist extends SubsystemBase {
  /** Creates a new AmpAssist. */
  Servo leftServo;
  Servo rightServo;
  
  public AmpAssist() {
    leftServo = new Servo(Constants.AmpAssistConstants.LEFT_SERVO_PORT);
    rightServo = new Servo(Constants.AmpAssistConstants.RIGHT_SERVO_PORT);
  }

  public Command ampAssistCommand() {
    return startEnd(()-> {
      leftServo.set(AmpAssistConstants.EXTEND_POSITION);
      rightServo.set(1-AmpAssistConstants.EXTEND_POSITION);
    }, ()-> {
      leftServo.set(AmpAssistConstants.RETRACT_POSITION);
      rightServo.set(1-AmpAssistConstants.RETRACT_POSITION);
    });

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("AmpAssist/leftServo", leftServo.get());
    SmartDashboard.putNumber("AmpAssist/rightServo", rightServo.get());

    // This method will be called once per scheduler run
  }
}
