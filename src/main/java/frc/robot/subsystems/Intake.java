// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.HeroSparkPID;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{

  private CANSparkMax intakeMotor;
  private CANSparkMax deployMotor;
  private RelativeEncoder intakeEncoder;
  private DigitalInput beamBreak;
  private HeroSparkPID deployController;
  private DigitalInput deployLimitSwitch;


  /* REV’s docs here (https://docs.revrobotics.com/through-bore-encoder/application-examples#ni-roborio) outline the different wiring options:
    If you use through bore encoder as a quadrature / relative encoder, use the Encoder class.
    If you use through bore encoder as a duty cycle / absolute encoder, use the DutyCycleEncoder class.
  If the SparkMax is controlling a brushless motor (NEO/NEO550), you would need to wire it for Alternate Encoder Mode 
    (https://docs.revrobotics.com/sparkmax/operating-modes/using-encoders/alternate-encoder-mode) and use getAlternateEncoder() */
    private SparkAbsoluteEncoder deployEncoder; 

  /* Creates a new GroundIntake. */
  public Intake() {
    intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);
    deployMotor = new CANSparkMax(IntakeConstants.deployMotorID, MotorType.kBrushless);
    beamBreak = new DigitalInput(IntakeConstants.beamBreakPort);
    deployLimitSwitch = new DigitalInput(IntakeConstants.deployLimitSwitchPort);
    

    deployEncoder = deployMotor.getAbsoluteEncoder(Type.kDutyCycle);
    deployController = new HeroSparkPID(deployMotor).useAbsoluteEncoder();
    deployController.setPID(IntakeConstants.deployPID);
    
    deployMotor.burnFlash();

    SmartDashboard.putData("Intake/subsystem",this);
    SmartDashboard.putData("Intake/pid",deployController);
  }

  @Override
  public void periodic() {
    // stolen from super
    SmartDashboard.putNumber("Intake/DeployRotations", this.getDeployRotation());

  }

  /* Intaking */
  public void runIntake() {
    intakeMotor.set(Constants.IntakeConstants.intakeSpeed);
  }

  public double getDeployRotation() {
    return deployEncoder.getPosition();
  }

  public void intakeNote() {

      intakeMotor.set(Constants.IntakeConstants.intakeSpeed);

  }

  public void outtakeNote() {
    intakeMotor.set(IntakeConstants.outtakeSpeed);
  }

  public void stopIntake() {
    intakeMotor.set(0);

  }

  /* via Chloe */
  public void setDeploySetPoint(double setpoint) {
    deployMotor.set(deployController.calculate(getDeployRotation(), setpoint));

    // deployController.setReference(setpoint, ControlType.kPosition);
  }

  public void setDeploySpeed (double speed) {
    deployMotor.set(speed);
  }

  public void stopDeploy() {
    deployMotor.set(0);
    deployController.setReference(0, ControlType.kPosition);

  }

  // TODO: this doesn't work no matter how much I want it to so lets fix that tmr


  public void coastMode() {
    deployMotor.setIdleMode(IdleMode.kCoast);
  }
  // should be working deploy once we get pid working

  public void retractIntake() {
    deployMotor.set(IntakeConstants.retractSpeed);
  }




  /* Commands */

  public Command deployIntakeCommand() {

    // return startEnd(() -> setDeploySetPoint(IntakeConstants.deployRotations), this::stopDeploy);//.until(() -> deployController.atSetpoint()).andThen(() -> deployMotor.setIdleMode(IdleMode.kCoast));
    return startEnd(()->this.setDeploySpeed(-0.2), this::stopDeploy).until(() -> getDeployRotation() <= IntakeConstants.deployRotations||!deployLimitSwitch.get());
  }

  public Command retractIntakeCommand() {
    // return startEnd(() -> setDeploySetPoint(IntakeConstants.retractRotations), this::stopDeploy);//.until(() ->  deployController.atSetpoint()).andThen(() -> deployMotor.setIdleMode(IdleMode.kBrake));
    return startEnd(()->this.setDeploySpeed(0.2), this::stopDeploy).until(() -> getDeployRotation() >= IntakeConstants.retractRotations);
  }

  public Command intakeNoteCommand() {
    return startEnd(() -> this.intakeNote(), this::stopIntake).until(()-> beamBreak.get());
  }

  public Command intakeNoteSequence() {
    return deployIntakeCommand().andThen(new PrintCommand("deploy done"), intakeNoteCommand()).andThen(new PrintCommand("intake done"), retractIntakeCommand());
  }

  public Command outtakeNoteCommand () {
    return startEnd(() -> this.outtakeNote(), ()-> this.stopIntake());
  }



}
