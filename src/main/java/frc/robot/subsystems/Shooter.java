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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ShooterConstants;
import frc.lib.HeroSparkPID;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  CANSparkMax leftKicker;
  CANSparkMax rightKicker;
  CANSparkMax leftFeederMotor;
  CANSparkMax rightFeederMotor;

  SysIdRoutine lShootRoutine;
  SysIdRoutine rShootRoutine;

  HeroSparkPID leftController;
  HeroSparkPID rightController;

  public Shooter() {
    leftKicker = new CANSparkMax(ShooterConstants.leftKickerMotorId, MotorType.kBrushless);
    rightKicker = new CANSparkMax(ShooterConstants.rightKickerMotorId, MotorType.kBrushless);

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
    SmartDashboard.putNumber("Shooter/leftSpeed", 6000);
    SmartDashboard.putNumber("Shooter/rightSpeed", -6000);

    SmartDashboard.putData("Shooter/subsystem",this);
    SmartDashboard.putData("Shooter/leftPID",leftController);
    SmartDashboard.putData("Shooter/rightPID",rightController);
    SmartDashboard.putData("Shooter/SysId/left/dynamic forward", lSysIdDynamic(Direction.kForward));
    SmartDashboard.putData("Shooter/SysId/left/dynamic backward", lSysIdDynamic(Direction.kReverse));
    SmartDashboard.putData("Shooter/SysId/left/quasistatic forward", lSysIdQuasistatic(Direction.kForward));
    SmartDashboard.putData("Shooter/SysId/left/quasistatic backward", lSysIdQuasistatic(Direction.kReverse));
    SmartDashboard.putData("Shooter/SysId/right/dynamic forward", rSysIdDynamic(Direction.kForward));
    SmartDashboard.putData("Shooter/SysId/right/dynamic backward", rSysIdDynamic(Direction.kReverse));
    SmartDashboard.putData("Shooter/SysId/right/quasistatic forward", rSysIdQuasistatic(Direction.kForward));
    SmartDashboard.putData("Shooter/SysId/right/quasistatic backward", rSysIdQuasistatic(Direction.kReverse));


  }
  public void lShootVoltage(Measure<Voltage> volt) {
    leftKicker.setVoltage(volt.in(Units.Volts));
  }
  public void rShootVoltage(Measure<Voltage> volt) {
    rightKicker.setVoltage(volt.in(Units.Volts));
  }
  public Command lSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return lShootRoutine.quasistatic(direction);
  }

  public Command lSysIdDynamic(SysIdRoutine.Direction direction) {
    return lShootRoutine.dynamic(direction);
  }
  public Command rSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return rShootRoutine.quasistatic(direction);
  }

  public Command rSysIdDynamic(SysIdRoutine.Direction direction) {
    return rShootRoutine.dynamic(direction);
  }



  public void setLeftKickerMotorSpeedRPM(double velocity) {
    leftController.setReference(velocity, ControlType.kVelocity);
    // leftKicker.set(velocity);  

  }

  public void setRightKickerMotorSpeedRPM(double velocity) {
    rightController.setReference(velocity, ControlType.kVelocity);
    // rightKicker.set(velocity);  

  }

  public void stopRightKickerMotor() {
    rightKicker.set(0);
  }

  public void stopLeftKickerMotor() {
    leftKicker.set(0);
  }

  public void stopKickerMotors() {
    stopLeftKickerMotor();
    stopRightKickerMotor(); 
  }

  private void setKickerSpeedsFromSmartDashboard() {
    //TODO: do math here
    setLeftKickerMotorSpeedRPM(SmartDashboard.getNumber("Shooter/leftSpeed", 0));
    setRightKickerMotorSpeedRPM(SmartDashboard.getNumber("Shooter/rightSpeed", 0));
  }

  public Command shootCommand() {
    return runEnd(this::setKickerSpeedsFromSmartDashboard, this::stopKickerMotors);
  }
  public boolean ready() {
    return leftController.atSetpoint() && rightController.atSetpoint();
  }
  public boolean hasShot() {
    //TODO: implement me
    return false;
  }

  /**
   * speed form 0 -1
   */
  public void setFeedMotorSpeed(double speed) {
    leftFeederMotor.set(speed);
  }
  public void stopFeedMotor() { 
    setFeedMotorSpeed(0);
  }

  public Command fullShooter(Intake intake) {
    return this.shootCommand()                    // shoot
            .alongWith(                           // as well as
                // new WaitUntilCommand(this::ready)  // wait for motor to get to speed
                // .withTimeout(3)             // or for 3 seconds to pass
                Commands.waitSeconds(.5)             // or for 3 seconds to pass
                .andThen(intake.outtakeNoteCommand())//then outtake into shooter
            )
            .until(this::hasShot)                   //until it has shot
            .withTimeout(4);                //or 2 seconds pass 
                                                    //then interrupt all commands, stopping outtake and shooter
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/leftRealSpeed",leftController.getSpeed());
    SmartDashboard.putNumber("Shooter/rightRealSpeed",rightController.getSpeed());

    // This method will be called once per scheduler run
  }
}