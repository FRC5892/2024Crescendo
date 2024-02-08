// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Add your docs here. */
public class HeroSparkPID implements Sendable {
    SparkPIDController controller;
    CANSparkBase spark;
    double p;
    double i;
    double d;
    double iZone;
    double reference;
    CANSparkBase.ControlType controlType;
    public HeroSparkPID(CANSparkBase spark) {

        this.spark = spark;
        this.controller = spark.getPIDController();
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

    public boolean atSetpoint() {
        //TODO: this might work?
        double velocity;
        if (controlType.equals(CANSparkBase.ControlType.kVelocity)) {
            velocity = Math.abs(spark.getEncoder().getVelocity());
        } else if (controlType.equals(CANSparkBase.ControlType.kPosition)) {
            velocity = Math.abs(spark.getEncoder().getPosition());
        } else {
            throw new RuntimeException("unimplemented");
        }
        double precision = reference * 0.05;
        return velocity+precision>=reference && velocity-precision<=reference;
    }


    public REVLibError setReference(double value, CANSparkBase.ControlType ctrl) {
        this.reference = value;
        this.controlType = ctrl;

        return controller.setReference(value, ctrl, 0, 0);
    }

    public final void setPID(PIDConstants pidConstants) {
        setP(pidConstants.kP);
        setI(pidConstants.kI);
        setD(pidConstants.kD);
    }

}
