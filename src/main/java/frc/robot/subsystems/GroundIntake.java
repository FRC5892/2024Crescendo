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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
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

  //TODO: do we even have a quadrature or hall sensor encoder on the shaft???
  public RelativeEncoder deployEncoder;

  /* Creates a new GroundIntake. */
  public GroundIntake() {
    intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);
    deployMotor = new CANSparkMax(IntakeConstants.deployMotorID, MotorType.kBrushless);
    
    
    //deploy
    deployEncoder = deployMotor.getEncoder();
    deployEncoder.setPosition(0);

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
  

  /* Deploying Intake via Michael*/ 
  // public void setDeploySpeed(Measure<Velocity<Angle>> velocity) {
  //   deployController.setReference(velocity.in(Units.RPM), ControlType.kVelocity);
  // }

  /* via Chloe */
  public void setDeploySetPoint(double setpoint) {
    setpoint = setpoint;
    deployController.setReference(setpoint, ControlType.kVelocity);
    System.out.println(setpoint);
  }

  public double getDeployVelocity() {
    return deployEncoder.getVelocity();
  }

  public boolean deployAtSetpoint() {
    return (Math.abs(setPoint - deployEncoder.getVelocity()) < IntakeConstants.deployRotations);
  }

  public void stopDeploy () {
    System.out.println("stopping");
    setDeploySetPoint(0);
  }

  //TODO: this doesn't work no matter how much I want it to so lets fix that tmr
  public void deployIntake() {
    double encoderPosition = deployEncoder.getPosition();
    boolean intakeDeployed = encoderPosition >= Constants.IntakeConstants.deployRotations;
    boolean intakeRetracted = encoderPosition <= 0;
    

    //if intake is not deployed run motor until 5 motor rotations
    if (intakeRetracted){
      //TODO: switch with PID
      deployMotor.set(Constants.IntakeConstants.deploySpeed);
    } 

    if (intakeDeployed) {
      stopDeploy();
    }
  }

  public void retractIntake() {
    deployMotor.set(Constants.IntakeConstants.retractSpeed);
  }

}
