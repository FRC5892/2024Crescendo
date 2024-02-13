// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class beambreakSendable implements Sendable{
    DigitalInput beambreak;
    public beambreakSendable(DigitalInput beambreak) {
        this.beambreak = beambreak;
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("status", beambreak::get, (boolean b) -> {});
    }
    
}
