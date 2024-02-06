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
import frc.lib.Utilities;
import frc.robot.Constants;

public class GroundIntake extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private CANSparkMax deployMotor;
  private SparkPIDController deployController;

  private RelativeEncoder deployEncoder;

  /* Creates a new GroundIntake. */
  public GroundIntake() {
    intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotorID, MotorType.kBrushless);
    deployMotor = new CANSparkMax(Constants.IntakeConstants.deployMotorID, MotorType.kBrushless);
    
    deployController = deployMotor.getPIDController();
    Utilities.setPID(deployController, Constants.IntakeConstants.deployPID);

    
    
    //deploy
    deployEncoder = deployMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }



  /* Intaking */
  public void runIntake() {
    intakeMotor.set(Constants.IntakeConstants.intakeSpeed);
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
