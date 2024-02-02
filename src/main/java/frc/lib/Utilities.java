// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.SparkPIDController;

/** Add your docs here. */
public class Utilities {
    public static final void setPID(SparkPIDController controller, PIDConstants pidConstants) {
        controller.setP(pidConstants.kP);
        controller.setI(pidConstants.kI);
        controller.setD(pidConstants.kD);
    }
}
