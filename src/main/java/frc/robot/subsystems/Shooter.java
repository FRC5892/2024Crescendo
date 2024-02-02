// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    leftController = leftSparkMax.getPIDController();
    rightController = rightSparkMax.getPIDController();
    Utilities.setPID(leftController,ShooterConstants.leftPID);
    Utilities.setPID(rightController,ShooterConstants.rightPID);

    SmartDashboard.putData("Shooter/leftPID",new SparkPIDSendable(leftController));
    SmartDashboard.putData("Shooter/rightPID",new SparkPIDSendable(rightController));

  }

  public void setLeftMotorSpeed(Measure<Velocity<Distance>> velocity) {
    Measure<Velocity<Angle>> angularVelocity = Units.RadiansPerSecond
        .of(velocity.in(Units.MetersPerSecond) / ShooterConstants.wheelDiameter.in(Units.Meters));
    leftController.setReference(angularVelocity.in(Units.RPM),ControlType.kVelocity);
  }

  public void setRightMotorSpeed() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
