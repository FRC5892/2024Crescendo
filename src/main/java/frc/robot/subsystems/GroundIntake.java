// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Utilities;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class GroundIntake extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private CANSparkMax deployMotor;
  private SparkPIDController deployController;
  private double setPoint;
  // private RelativeEncoder deployEncoder;
  public DutyCycleEncoder deployEncoder;

  //Through Bore encoder btw

  /* Creates a new GroundIntake. */
  public GroundIntake() {
    intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);
    deployMotor = new CANSparkMax(IntakeConstants.deployMotorID, MotorType.kBrushless);
    deployEncoder = new DutyCycleEncoder(2);
    deployEncoder.reset();
    // deployEncoder = deployMotor.getAlternateEncoder(Type.kQuadrature, 8192); 

    deployController = deployMotor.getPIDController();
    deployController.setP(IntakeConstants.deployPIDF[0]);
    deployController.setI(IntakeConstants.deployPIDF[1]);
    deployController.setD(IntakeConstants.deployPIDF[2]);
    deployController.setFF(IntakeConstants.deployPIDF[3]);
    deployMotor.burnFlash();

    SmartDashboard.putNumber("Deploy P", IntakeConstants.deployPIDF[0]);
    SmartDashboard.putNumber("Deploy I", IntakeConstants.deployPIDF[1]);
    SmartDashboard.putNumber("Deploy D", IntakeConstants.deployPIDF[2]);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("DeployRotations", deployEncoder.getDistance());

    // This method will be called once per scheduler run
  }



  /* Intaking */
  public void runIntake() {
    intakeMotor.set(Constants.IntakeConstants.intakeSpeed);
  }

  public void outtakeNote() {
    intakeMotor.set(-IntakeConstants.intakeSpeed);
  }

  public void stopIntake() {
        System.out.println("stopping");
        intakeMotor.set(0);
    ;
  }

  /* via Chloe */
  public void setDeploySetPoint(double setpoint) {
    this.setPoint = setpoint;
    deployController.setReference(setpoint, ControlType.kVelocity);
    System.out.println(setpoint);
  }

  public void stopDeploy () {
    System.out.println("stopping");
    deployMotor.set(0);
  }

  //TODO: this doesn't work no matter how much I want it to so lets fix that tmr
  public void deployIntake() {
    double encoderPosition = deployEncoder.getDistance();
    boolean intakeDeployed = encoderPosition >= IntakeConstants.deployRotations;
    boolean intakeRetracted = encoderPosition <= IntakeConstants.retractRotations;
    

    //if intake is not deployed run motor until 5 motor rotations
    if (intakeRetracted){
      //TODO: switch with PID
      deployMotor.set(IntakeConstants.deploySpeed);
    } 

    if (intakeDeployed) {
      stopDeploy();
    }
  }

  public void retractIntake() {
    
    deployMotor.set(IntakeConstants.retractSpeed);

    // double encoderPosition = deployEncoder.getDistance();
    // boolean intakeDeployed = encoderPosition >= 0.6;
    // boolean intakeRetracted = encoderPosition <= 0.2;

    // if (intakeDeployed) {
    //   deployMotor.set(IntakeConstants.retractSpeed);
    // } else if (intakeRetracted){
    //   //TODO: switch with PID
    //   stopDeploy();
    // } 
    

  }

  public Command outtakeNoteCommand () {
    return startEnd(() -> this.outtakeNote(), ()-> this.stopIntake());
  }

}
