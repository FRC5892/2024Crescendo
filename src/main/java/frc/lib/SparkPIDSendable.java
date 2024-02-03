// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Add your docs here. */
public class SparkPIDSendable implements Sendable {
    SparkPIDController controller;
    public SparkPIDSendable (SparkPIDController controller) {
        this.controller =  controller;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PIDController");
    builder.addDoubleProperty("p", controller::getP, controller::setP);
    builder.addDoubleProperty("i", controller::getI, controller::setI);
    builder.addDoubleProperty("d", controller::getD, controller::setD);
    builder.addDoubleProperty(
        "izone",
        controller::getIZone,
        (double toSet) -> {
          try {
            controller.setIZone(toSet);
          } catch (IllegalArgumentException e) {
            MathSharedStore.reportError("IZone must be a non-negative number!", e.getStackTrace());
          }
        });
        //FIXME: how do we get setpoint?
    builder.addDoubleProperty("setpoint", ()->0, (double d)->{});
    // pid setter
    //(double s)-> controller.setReference(s, CANSparkBase.ControlType.kVelocity)
    }
    
    
}
