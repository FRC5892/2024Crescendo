// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.SparkPIDSendable;
import frc.lib.Utilities;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  CANSparkMax leftSparkMax;
  CANSparkMax rightSparkMax;
  SparkPIDController leftController;
  SparkPIDController rightController;

  public Shooter() {
    leftSparkMax = new CANSparkMax(ShooterConstants.leftMotorId, MotorType.kBrushless);
    rightSparkMax = new CANSparkMax(ShooterConstants.rightMotorId, MotorType.kBrushless);

    // setup Pid
    leftController = leftSparkMax.getPIDController();
    rightController = rightSparkMax.getPIDController();
    Utilities.setPID(leftController, ShooterConstants.leftPID);
    Utilities.setPID(rightController, ShooterConstants.rightPID);
    SmartDashboard.putData("Shooter/leftPID", new SparkPIDSendable(leftController));
    SmartDashboard.putData("Shooter/rightPID", new SparkPIDSendable(rightController));
    SmartDashboard.putData("Shooter/subsystem",this);

  }

  public void setLeftMotorSpeed(Measure<Velocity<Distance>> velocity) {
    setMotorSpeedFromLinearVelocity(rightController, velocity);
  }

  public void setRightMotorSpeed(Measure<Velocity<Distance>> velocity) {
    setMotorSpeedFromLinearVelocity(leftController, velocity);
  }

  public void stopRightMotor() {
    setRightMotorSpeed(Units.MetersPerSecond.of(0));
  }

  public void stopLeftMotor() {
    setLeftMotorSpeed(Units.MetersPerSecond.of(0));
  }

  public void stopMotors() {
    stopLeftMotor();
    stopRightMotor();
  }

  public void setMotorSpeedFromLinearVelocity(SparkPIDController controller,
      Measure<Velocity<Distance>> linearVelocity) {
    // Linear Velocity to RPM
    Measure<Velocity<Angle>> angularVelocity = Units.RadiansPerSecond
        .of(linearVelocity.in(Units.MetersPerSecond) / ShooterConstants.wheelDiameter.in(Units.Meters));
    controller.setReference(angularVelocity.in(Units.RPM), ControlType.kVelocity);
  }
  private void setSpeedFromSmartDashboard() {
    setLeftMotorSpeed(Units.MetersPerSecond.of(SmartDashboard.getNumber("Shooter/leftSpeed", 0)));
    setRightMotorSpeed(Units.MetersPerSecond.of(SmartDashboard.getNumber("Shooter/rightSpeed", 0)));
  }

  public Command shootCommand() {
    return runEnd(this::setSpeedFromSmartDashboard, this::stopLeftMotor);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
