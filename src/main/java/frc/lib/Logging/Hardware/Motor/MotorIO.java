package frc.lib.Logging.Hardware.Motor;

import org.littletonrobotics.junction.AutoLog;

import com.pathplanner.lib.util.PIDConstants;

public interface MotorIO {
    @AutoLog
    public static class MotorIOInputs {
        public double positionRot = 0.0;
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
    }
    public default void updateInputs(MotorIOInputs inputs){};

    public default void setPID(PIDConstants constants){};

    public default void set(double speed){};

    public default void setVelocity(double velocityRPM){};

    public default void setVoltage(double voltageVolts){};

    public default void stop(){};


    public default void setPosition(double position){};
}
