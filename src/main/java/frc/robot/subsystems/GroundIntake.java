// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GroundIntake extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private CANSparkMax deployMotor;
  private RelativeEncoder deployEncoder;

  /** Creates a new GroundIntake. */
  public GroundIntake() {
    intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotorID, MotorType.kBrushless);
    deployMotor = new CANSparkMax(Constants.IntakeConstants.deployMotorID, MotorType.kBrushless);
    deployEncoder = deployMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void stopIntake() {
    setIntakeSpeed(0);
  }
  /* runs intake at intakeSpeed until interrupted 
   */
  public Command intakePieceCommand() {
    return startEnd(()-> this.setIntakeSpeed(Constants.IntakeConstants.intakeSpeed), ()->this.stopIntake());
  }

  public void deployIntake () {
    if (true) {
      
    }
    deployMotor.set(Constants.IntakeConstants.deploySpeed);
  }

  public void retractIntake () {

  }
}
