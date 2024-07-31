package frc.lib.Logging.assignment;

public class HardwareMotor {
    private int bot;
    private int id;
    private MotorType type;
    enum MotorType {
        CAN_SPARK_MAX,
        TALON_FX
    }
    public int getBot() {
        return bot;
    }
    public MotorType getMotorType() {
        return type;
    }
    public int getId() {
        return id;
    }

}
