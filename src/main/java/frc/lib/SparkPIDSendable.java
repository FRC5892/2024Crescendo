// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import com.revrobotics.SparkPIDController;

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
        builder.addDoubleProperty("p", controller::getP, controller::setP);
        builder.addDoubleProperty("i", controller::getI, controller::setI);
        builder.addDoubleProperty("d", controller::getD, controller::setD);
    }
    
}
