// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.HeroSparkPID;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private CANSparkMax deployMotor;
  private RelativeEncoder intakeEncoder;
  private HeroSparkPID deployController;
  private DigitalInput beamBreak;

  /* REV’s docs here (https://docs.revrobotics.com/through-bore-encoder/application-examples#ni-roborio) outline the different wiring options:
    If you use through bore encoder as a quadrature / relative encoder, use the Encoder class.
    If you use through bore encoder as a duty cycle / absolute encoder, use the DutyCycleEncoder class.
  If the SparkMax is controlling a brushless motor (NEO/NEO550), you would need to wire it for Alternate Encoder Mode 
    (https://docs.revrobotics.com/sparkmax/operating-modes/using-encoders/alternate-encoder-mode) and use getAlternateEncoder() */
    private DutyCycleEncoder deployEncoder; 

  /* Creates a new GroundIntake. */
  public Intake() {
    intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);
    deployMotor = new CANSparkMax(IntakeConstants.deployMotorID, MotorType.kBrushless);
    beamBreak = new DigitalInput(2);
    deployEncoder = new DutyCycleEncoder(IntakeConstants.beamBreakPort);
    deployEncoder.reset();
    // deployEncoder = deployMotor.getAlternateEncoder(Type.kQuadrature, 8192);

    deployController = new HeroSparkPID(deployMotor);

    deployController.setPID(IntakeConstants.deployPID);
    SmartDashboard.putData(deployController);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("DeployRotations", this.getDeployRotation());
  }

  /* Intaking */
  public void runIntake() {
    intakeMotor.set(Constants.IntakeConstants.intakeSpeed);
  }

  public double getDeployRotation() {
    return deployEncoder.getDistance();
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
    deployController.setReference(setpoint, ControlType.kPosition);
    System.out.println(setpoint);
  }

  public void stopDeploy() {
    deployMotor.set(0);
  }

  // TODO: this doesn't work no matter how much I want it to so lets fix that tmr
  public void deployIntake() {


    deployMotor.set(IntakeConstants.deploySpeed);
  }
  // should be working deploy once we get pid working
  // public Command deployCommand() {
  //   return runEnd(() -> setDeploySetPoint(IntakeConstants.deployRotations),this::stopDeploy).until(deployController::atSetpoint);
    
  // }

  public void retractIntake() {
    deployMotor.set(IntakeConstants.retractSpeed);
  }




  /* Commands */

  public Command deployIntakeCommand() {
    return startEnd(() -> {deployMotor.set(IntakeConstants.deploySpeed);}, this::stopDeploy).until(() -> getDeployRotation() >= 0.5);
  }

  public Command retractIntakeCommand() {
    return startEnd(() -> {deployMotor.set(IntakeConstants.retractSpeed);}, this::stopDeploy).until(() -> getDeployRotation() <= 0.2);
  }

  public Command intakeNoteCommand() {
    return startEnd(() -> this.intakeNote(), () -> this.stopIntake()).until(() -> beamBreak.get());
  }

  public Command intakeNoteSequence() {
    
    return deployIntakeCommand().andThen(new PrintCommand("deploy done"), intakeNoteCommand()).andThen(new PrintCommand("intake done"), retractIntakeCommand());
  }

  public Command outtakeNoteCommand() {
    return startEnd(() -> this.outtakeNote(), () -> this.stopIntake());
  }


}
