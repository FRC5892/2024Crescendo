// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.SparkPIDSendable;
import frc.lib.Utilities;
import frc.robot.Constants;

public class GroundIntake extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private CANSparkMax deployMotor;
  private SparkPIDController deployController;

  private RelativeEncoder deployEncoder;
  private RelativeEncoder intakeEncoder;

  /* Creates a new GroundIntake. */
  public GroundIntake() {
    intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotorID, MotorType.kBrushless);
    deployMotor = new CANSparkMax(Constants.IntakeConstants.deployMotorID, MotorType.kBrushless);
    
    deployController = deployMotor.getPIDController();
    Utilities.setPID(deployController, Constants.IntakeConstants.deployPID);
    // SmartDashboard.putData("Intake/intakePID", new SparkPIDSendable(intakeController));
    // SmartDashboard.putData("Intake/deployPID", new SparkPIDSendable(deployController));
    
    /* Encoders */
    deployEncoder = deployMotor.getEncoder();
    intakeEncoder = intakeMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }



  /* Intaking */
  public void runIntake() {
    intakeMotor.set(Constants.IntakeConstants.intakeSpeed);
  }

  public void intakeNote() {
    //TODO: add sensors
    intakeEncoder.setPosition(0);
    double encoderPosition = intakeEncoder.getPosition();
    boolean noteIntaked = encoderPosition >= Constants.IntakeConstants.intakeRotations;
    
    //if intake is not deployed run motor until 5 motor rotations
    if (!noteIntaked){
      intakeMotor.set(Constants.IntakeConstants.intakeSpeed);
    } else if (noteIntaked) {
      stopDeploy();
    }
  }

  public void stopIntake() {
        System.out.println("stopping");
        intakeMotor.set(0);
    ;
  }
  

  /* Deploying Intake */ 
  public void setDeploySpeed(double speed) {
    // deployController.setReference(speed, ControlType.kVelocity);
    deployMotor.set(speed);
  }

  public void stopDeploy () {
    System.out.println("stopping");
    setDeploySpeed(0);
  }

  public void deployIntake() {
    deployEncoder.setPosition(0);
    double encoderPosition = deployEncoder.getPosition();
    boolean intakeDeployed = encoderPosition >= Constants.IntakeConstants.deployRotations;
    
    //if intake is not deployed run motor until 5 motor rotations
    if (!intakeDeployed){
      deployMotor.set(Constants.IntakeConstants.deploySpeed);
    } else if (intakeDeployed) {
      stopDeploy();
    }
  }

  public void retractIntake() {
    double encoderPosition = deployEncoder.getPosition();
    boolean intakeDeployed = encoderPosition >= Constants.IntakeConstants.deployRotations;
    boolean intakeRetracted = encoderPosition <= 0;

    //if intake is deployed run motor 5 motor rotations backwards
    if (intakeDeployed) {
      deployMotor.set(Constants.IntakeConstants.retractSpeed);
    } else if (intakeRetracted) {
      stopDeploy();
    }
  }


  /* Testing Commands */
  public Command intakeNoteCommand() {
    return startEnd(()-> this.runIntake(), ()->this.stopIntake());
  }

  public Command retractIntakeCommand() {
    return startEnd(()-> this.setDeploySpeed(Constants.IntakeConstants.retractSpeed), ()-> this.stopDeploy());
  }

  public Command deployIntakeCommand() {
    return startEnd(()-> this.setDeploySpeed(Constants.IntakeConstants.deploySpeed), ()-> this.stopDeploy());
  }
}
