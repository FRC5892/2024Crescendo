// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/* 
Summary:
This code is for the robot container and has a joy stick, joystick buttons, swerve subsystem, a sendable chooser for autonomous modes, autonomous modes, and methods for configuring button bindings and smart dashboard options. 
*/

public class RobotContainer {
        /* Controllers */
                public final static Joystick driver = new Joystick(0);
                // private final Joystick coDriver = new Joystick(1);

        /* Subsystems & Hardware */
                //TODO: add compressor when we have a compressor 
                /* Compressor */
                // private Compressor compressor;
                /* Gyro Sensor */
                private static Pigeon2 gyro = new Pigeon2(Constants.Swerve.pigeonID);
                /* Swerve Subsystem */
                private final Swerve s_Swerve = new Swerve(gyro);
                // private final LedSub ledSub = new LedSub();

        /* Controls and buttons */
                /* Drive Controls */
                private static final int translationAxis = XboxController.Axis.kLeftY.value;
                private static final int strafeAxis = XboxController.Axis.kLeftX.value;
                private static final int rotationAxis = XboxController.Axis.kRightX.value;
                private double SPEED_MULTIPLIER = 1.0;

                
                /* Driver Buttons */
                private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
                private final JoystickButton robotCentric = new JoystickButton(driver,
                        XboxController.Button.kRightBumper.value);
        /*Commands */
        
        /* Other */
                /* SendableChooser */
                public final SendableChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                /* Hardware and Logging */
                        // TODO: add Camera when we have a camera
                        // CameraServer.startAutomaticCapture();
                        
                        // TODO: add compressor when we have a compressor 
                        // compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
                        // compressor.enableDigital();


                /* Default Commands */

                        s_Swerve.setDefaultCommand(
                                new TeleopSwerve(
                                        s_Swerve,

                                        () -> -driver.getRawAxis(translationAxis) * SPEED_MULTIPLIER,
                                        () -> -driver.getRawAxis(strafeAxis) * SPEED_MULTIPLIER,
                                        () -> -driver.getRawAxis(rotationAxis) * SPEED_MULTIPLIER,
                                        () -> robotCentric.getAsBoolean()));
                
                                        
                                        
                /* Others */
                        // Auto chooser
                        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
                        // Configure the button bindings
                        configureButtonBindings();

                        // Configure Smart Dashboard options
                        configureSmartDashboard();

                
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                /* Driver Buttons */
                zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        }

        private void configureSmartDashboard() {
                SmartDashboard.putNumber("Speed Multiplier", SPEED_MULTIPLIER);
                SmartDashboard.putData("Auto Chooser", autoChooser);

        
        }

        public void disabledInit() {
                s_Swerve.resetToAbsolute();
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // Executes the autonomous command chosen in smart dashboard
                return autoChooser.getSelected();

        }
}