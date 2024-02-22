// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Add your docs here. */
public class HeroSparkPID implements Sendable {
    SparkPIDController controller;
    CANSparkBase spark;
    RelativeEncoder encoder;
    SparkAbsoluteEncoder absoluteEncoder;
    boolean isAbsolute;
    double p;
    double i;
    double d;
    double iZone;
    double ff;
    double reference;
    CANSparkBase.ControlType controlType;

    double setpoint;
    double measurement;
    double prevError;
    double positionError;
    double totalError;
    double period = 0.02;
    double velocityError;


    public HeroSparkPID(CANSparkBase spark) {

        this.spark = spark;
        this.encoder = spark.getEncoder();
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
    private double getFF() {
        return ff;
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
    private void setFF(double newFF) {
        if (newFF != ff) {
            controller.setIZone(newFF);
            ff = newFF;
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
        builder.addDoubleProperty("setpoint", this::getFF, this::setFF);
        // pid setter
        // (double s)-> controller.setReference(s, CANSparkBase.ControlType.kVelocity)
    }

    public boolean atSetpoint() {
        // TODO: this might work?
        double velocity;
        if (controlType.equals(CANSparkBase.ControlType.kVelocity)) {
            velocity = Math.abs(getSpeed());
        } else if (controlType.equals(CANSparkBase.ControlType.kPosition)) {
            velocity = Math.abs(getPosition());
        } else {
            throw new RuntimeException("unimplemented");
        }
        double precision = reference * 0.05;
        return velocity + precision >= reference && velocity - precision <= reference;
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

    public SparkPIDController getPIDController() {
        
        return controller;
    }

    public double getSpeed() {
        return isAbsolute ? absoluteEncoder.getVelocity() : encoder.getVelocity();
    }

    public double getPosition() {
        return isAbsolute ? absoluteEncoder.getPosition() : encoder.getPosition();
    }
    public double getReference() {
        return reference;
    }

    public HeroSparkPID useAbsoluteEncoder() {
        absoluteEncoder = spark.getAbsoluteEncoder(Type.kDutyCycle);
        controller.setFeedbackDevice(absoluteEncoder);
        return this;
    }

    /**
    * Returns the next output of the PID controller.
    *
    * @param measurement The current measurement of the process variable.
    * @param setpoint The new setpoint of the controller.
    * @return The next controller output.
    */
    public double calculate(double measurement, double setpoint) {
        //i think this works?? - chloe
        this.setpoint = setpoint;
        this.measurement = measurement;
        prevError = positionError;


        positionError = MathUtil.inputModulus(setpoint - measurement, -1, 1);

        velocityError = (positionError - prevError) / period;

        totalError =
          MathUtil.clamp(
              totalError + positionError * 0.02,
              -1 / i,
              1 / i);

        return p * positionError + i * totalError + d * velocityError;
    }

}
