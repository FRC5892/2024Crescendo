// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.HeroSparkPID;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class GroundIntake extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private CANSparkMax deployMotor;
  RelativeEncoder intakeEncoder;
  private HeroSparkPID deployController;
  private double setPoint = 0;

  //Through Bore encoder btw
  public DutyCycleEncoder deployEncoder;

  /* Creates a new GroundIntake. */
  public GroundIntake() {
    intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotorID, MotorType.kBrushless);
    deployMotor = new CANSparkMax(Constants.IntakeConstants.deployMotorID, MotorType.kBrushless);
    deployController = new HeroSparkPID(deployMotor);
    deployController.setPID(Constants.IntakeConstants.deployPID);
    intakeEncoder = intakeMotor.getEncoder();

    SmartDashboard.putData("Intake/deployPID", deployController);

    
    
    //deploy
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
        intakeMotor.set(0);

  }
  

  /* Deploying Intake via Michael*/ 
  // public void setDeploySpeed(Measure<Velocity<Angle>> velocity) {
  //   deployController.setReference(velocity.in(Units.RPM), ControlType.kVelocity);
  // }

  /* via Chloe */
  public void setDeploySetPoint(double setpoint) {
    this.setPoint = setpoint;
    deployController.setReference(setpoint, ControlType.kVelocity);
    System.out.println(setpoint);
  }

  public void stopDeploy () {
    System.out.println("stopping");
    setDeploySetPoint(0);
  }

  //TODO: this doesn't work no matter how much I want it to so lets fix that tmr
  public void deployIntake() {
    double encoderPosition = deployEncoder.get();
    boolean intakeDeployed = encoderPosition >= Constants.IntakeConstants.deployRotations;
    boolean intakeRetracted = encoderPosition <= 0;
    

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
    double encoderPosition = deployEncoder.get();
    boolean intakeDeployed = encoderPosition >= IntakeConstants.deployRotations;
    boolean intakeRetracted = encoderPosition <= 0;
    

    //if intake is not deployed run motor until 5 motor rotations
    if (intakeRetracted){
      //TODO: switch with PID
      stopDeploy();
    }
  }


  /* Testing Commands */
  public Command intakeNoteCommand() {
    return run(this::deployIntake)
          .alongWith(new WaitCommand(1))
          .andThen(run(this::intakeNote)
                  .alongWith(new WaitCommand(1)))
          .andThen(run(this::retractIntake))
          .alongWith(new WaitCommand(1));
  }

  public Command retractIntakeCommand() {
    return startEnd(()-> this.retractIntake(), ()-> this.stopDeploy());
  }

  public Command deployIntakeCommand() {
    return startEnd(()-> this.deployIntake(), ()-> this.stopDeploy());
  }
}
