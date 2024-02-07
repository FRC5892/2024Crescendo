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
    double p;
    double i;
    double d;
    double iZone;

    public SparkPIDSendable(SparkPIDController controller) {
        this.controller = controller;
        p = controller.getP();
        i = controller.getI();
        d = controller.getD();
        iZone = controller.getIZone();

    }

    private double getP() {
        return p;
    }

    private double getI() {
        return i;
    }

    private double getD() {
        return d;
    }
    private double getIZone() {
        return iZone;
    }

    private void setP(double newP) {
        if (newP != p) {
            controller.setP(newP);
            p = newP;
        }   
    }
    private void setI(double newI) {
        if (newI != i) {
            controller.setI(newI);
            i = newI;
        }   
    }
    private void setD(double newD) {
        if (newD != d) {
            controller.setD(newD);
            d = newD;
        }   
    }

    private void setIZone(double newIZone) {
        if (newIZone != iZone) {
            controller.setIZone(newIZone);
            iZone = newIZone;
        }   
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty(
                "izone",
                this::getIZone,
                (double toSet) -> {
                    try {
                        this.setIZone(toSet);
                    } catch (IllegalArgumentException e) {
                        MathSharedStore.reportError("IZone must be a non-negative number!", e.getStackTrace());
                    }
                });
        // FIXME: how do we get setpoint?
        builder.addDoubleProperty("setpoint", () -> 0, (double d) -> {
        });
        // pid setter
        // (double s)-> controller.setReference(s, CANSparkBase.ControlType.kVelocity)
    }

}
