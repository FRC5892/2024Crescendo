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
  private SparkPIDController intakeController;
  private SparkPIDController deployController;

  private RelativeEncoder deployEncoder;

  /** Creates a new GroundIntake. */
  public GroundIntake() {
    intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotorID, MotorType.kBrushless);
    deployMotor = new CANSparkMax(Constants.IntakeConstants.deployMotorID, MotorType.kBrushless);
    
    intakeController = intakeMotor.getPIDController();
    deployController = deployMotor.getPIDController();
    Utilities.setPID(intakeController, Constants.IntakeConstants.intakePID);
    Utilities.setPID(deployController, Constants.IntakeConstants.deployPID);
    SmartDashboard.putData("Intake/intakePID", new SparkPIDSendable(intakeController));
    SmartDashboard.putData("Intake/deployPID", new SparkPIDSendable(deployController));

    
    
    //deploy
    deployEncoder = deployMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }



  /* Intaking */
  public void setIntakeSpeed(double speed) {
    intakeController.setReference(speed, ControlType.kVelocity);
  }

  public void stopIntake() {
    setIntakeSpeed(0);
  }
  

  /* Deploying Intake */ 
  public void setDeploySpeed(double speed) {
    deployController.setReference(speed, ControlType.kVelocity);

  }
  public void deployIntake () {
    setDeploySpeed(Constants.IntakeConstants.deploySpeed);
  }

  public void retractIntake () {
    setDeploySpeed(Constants.IntakeConstants.retractSpeed);
  }
  
  public void stopDeploy () {
    setDeploySpeed(0);
  }


  /* Testing Commands */
  public Command intakeNoteCommand() {
    return startEnd(()-> this.setIntakeSpeed(Constants.IntakeConstants.intakeSpeed), ()->this.stopIntake());
  }

  public Command retractIntakeCommand() {
    return startEnd(()-> this.retractIntake(), ()-> this.stopDeploy());
  }

  public Command deployIntakeCommand() {
    return startEnd(()-> this.deployIntake(), ()-> this.stopDeploy());
  }
}
