// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class BooleanButton extends Trigger {
	/**
	 * Create a gamepad axis for triggering commands as if it were a button.
	 *
	 * @param joystick     The GenericHID object that has the axis (e.g. Joystick, KinectStick,
	 *                     etc)
	 * @param axisNumber The axis number (see {@link GenericHID#getRawAxis(int) }
	 * 
	 * @param threshold The threshold above which the axis shall trigger a command
	 */
	public BooleanButton(BooleanSupplier bs) {
		super(bs);
	}
}
