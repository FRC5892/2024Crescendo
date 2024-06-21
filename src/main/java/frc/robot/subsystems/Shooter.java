// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import monologue.Logged;
import monologue.Annotations.Log;
import frc.lib.AutoManager;
import frc.lib.HeroSparkPID;

public class Shooter extends SubsystemBase implements Logged{
  CANSparkMax leftKicker;
  CANSparkMax rightKicker;
  CANSparkMax leftFeederMotor;
  CANSparkMax rightFeederMotor;

  SysIdRoutine lShootRoutine;
  SysIdRoutine rShootRoutine;
  
  @Log HeroSparkPID leftController;
  @Log HeroSparkPID rightController;

  public Shooter() {
    leftKicker = new CANSparkMax(ShooterConstants.LEFT_KICKER_MOTOR_ID, MotorType.kBrushless);
    rightKicker = new CANSparkMax(ShooterConstants.RIGHT_KICKER_MOTOR_ID, MotorType.kBrushless);

    // rightFeederMotor.follow(leftFeederMotor, false);
    lShootRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(this::lShootVoltage, null, this));
    rShootRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(this::rShootVoltage, null, this));
    // setup Pid
    leftController = new HeroSparkPID(leftKicker);
    rightController = new HeroSparkPID(rightKicker);
    // leftController.setPID(ShooterConstants.leftPID);
    // rightController.setPID(ShooterConstants.rightPID);
    this.log("leftSpeed", 6000);
    this.log("rightSpeed", -6000);

    AutoManager.addSysidCharacterization("Left Shooter", lShootRoutine);
    AutoManager.addSysidCharacterization("Right Shooter", rShootRoutine);
  }
  
  public void lShootVoltage(Measure<Voltage> volt) {
    leftKicker.setVoltage(volt.in(Units.Volts));
  }
  public void rShootVoltage(Measure<Voltage> volt) {
    rightKicker.setVoltage(volt.in(Units.Volts));
  }



  public void setLeftKickerMotorSpeedRPM(double velocity) {
    leftController.setReference(velocity, ControlType.kVelocity);
    // leftKicker.set(velocity);  

  }

  public void setRightKickerMotorSpeedRPM(double velocity) {
    rightController.setReference(velocity, ControlType.kVelocity);
    // rightKicker.set(velocity);  

  }

  public void stopKickerMotors() {
    rightKicker.set(0);
    leftKicker.set(0);
  }

  private void setKickerSpeedsFromSmartDashboard() {
    // setLeftKickerMotorSpeedRPM(logger.get("leftSpeed", 6000));
    // setRightKickerMotorSpeedRPM(logger.get("rightSpeed", -6000));
  }

  public Command shootCommand() {
    return runEnd(this::setKickerSpeedsFromSmartDashboard, this::stopKickerMotors);
  }

  public void setFeedMotorSpeed(double speed) {
    leftFeederMotor.set(speed);
  }

  public void stopFeedMotor() { 
    setFeedMotorSpeed(0);
  }

  /**
   * Command to hand off a NOTE into the Shooter and to shoot it.
   * <p>Runs for 1 second.
   * @param intake - the Intake subsystem
   */
  public Command fullShooter(Intake intake) {
    return this.shootCommand()
      .alongWith(
        Commands.waitSeconds(.5)
        .andThen(intake.outtakeNoteCommand())
      )
      .withTimeout(1);
  }

  @Override
  public void periodic() {
    this.log("leftRealSpeed",leftController.getSpeed());
    this.log("rightRealSpeed",rightController.getSpeed());

    // This method will be called once per scheduler run
  }
}