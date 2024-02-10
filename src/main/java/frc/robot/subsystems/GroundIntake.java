// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.HeroSparkPID;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class GroundIntake extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private CANSparkMax deployMotor;
  RelativeEncoder intakeEncoder;
  private HeroSparkPID deployController;
  // private RelativeEncoder deployEncoder;
  public DutyCycleEncoder deployEncoder;

  // Through Bore encoder btw

  /* Creates a new GroundIntake. */
  public GroundIntake() {
    intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);
    deployMotor = new CANSparkMax(IntakeConstants.deployMotorID, MotorType.kBrushless);
    deployEncoder = new DutyCycleEncoder(2);
    deployEncoder.reset();
    // deployEncoder = deployMotor.getAlternateEncoder(Type.kQuadrature, 8192);

    deployController = new HeroSparkPID(deployMotor);

    deployController.setPID(IntakeConstants.deployPID);
    SmartDashboard.putData(deployController);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("DeployRotations", deployEncoder.getDistance());

  }

  /* Intaking */
  public void runIntake() {
    intakeMotor.set(Constants.IntakeConstants.intakeSpeed);
  }

  public void intakeNote() {
    // TODO: add sensors

    intakeEncoder.setPosition(0);
    double encoderPosition = intakeEncoder.getPosition();
    boolean noteIntaked = encoderPosition >= Constants.IntakeConstants.intakeRotations;

    // if intake is not deployed run motor until 5 motor rotations
    if (!noteIntaked) {
      intakeMotor.set(Constants.IntakeConstants.intakeSpeed);
    } else if (noteIntaked) {
      stopDeploy();
    }

  }

  public void outtakeNote() {
    intakeMotor.set(-IntakeConstants.intakeSpeed);
  }

  public void stopIntake() {
    intakeMotor.set(0);

  }

  /* via Chloe */
  public void setDeploySetPoint(double setpoint) {
    // this.setPoint = setpoint;
    deployController.setReference(setpoint, ControlType.kVelocity);
    System.out.println(setpoint);
  }

  public void stopDeploy() {
    System.out.println("stopping");
    deployMotor.set(0);
  }

  // TODO: this doesn't work no matter how much I want it to so lets fix that tmr
  public void deployIntake() {
    double encoderPosition = deployEncoder.getDistance();
    boolean intakeDeployed = encoderPosition >= IntakeConstants.deployRotations;
    boolean intakeRetracted = encoderPosition <= IntakeConstants.retractRotations;

    // if intake is not deployed run motor until 5 motor rotations
    if (intakeRetracted) {
      // TODO: switch with PID
      deployMotor.set(IntakeConstants.deploySpeed);
    }

    if (intakeDeployed) {
      stopDeploy();
    }
  }

  public void retractIntake() {

    deployMotor.set(IntakeConstants.retractSpeed);

    double encoderPosition = deployEncoder.getDistance();
    boolean intakeDeployed = encoderPosition >= 0.6;
    boolean intakeRetracted = encoderPosition <= 0.2;

    // if intake is not deployed run motor until 5 motor rotations
    if (intakeRetracted) {
      // TODO: switch with PID
      stopDeploy();
    }

    if (intakeDeployed) {
      deployMotor.set(IntakeConstants.retractSpeed);
    }
  }
    public Command outtakeNoteCommand () {
    return startEnd(() -> this.outtakeNote(), ()-> this.stopIntake());
  }

  // public void retractIntake() {
  // deployMotor.set(Constants.IntakeConstants.retractSpeed);
  // }

}
