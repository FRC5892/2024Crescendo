// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Logging.assignment;

import java.util.HashMap;
import java.util.Map;

import frc.robot.Hardware;

/** Add your docs here. */
public class SoftwareMotor {
    final Map<Integer,HardwareMotor> hardwareMotors;
    public SoftwareMotor(Map<Integer,HardwareMotor> HardwareMotors) {
        hardwareMotors = HardwareMotors;
    }
    public SoftwareMotor(HardwareMotor... hardwareMotors) {
        this.hardwareMotors = new HashMap<Integer,HardwareMotor>();
        for (HardwareMotor motor : hardwareMotors) {
            this.hardwareMotors.put(motor.getBot(), motor);
            
        }
    }
    public HardwareMotor getMotor() {
        return hardwareMotors.get(Hardware.bot);
    }
}
