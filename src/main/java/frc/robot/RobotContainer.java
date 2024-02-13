// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.Intake.DeployIntake;
import frc.robot.commands.Intake.IntakeNote;
import frc.robot.commands.Intake.IntakeNoteSequence;
import frc.robot.commands.Intake.RetractIntake;
import frc.robot.subsystems.*;

/* 
Summary:
This code is for the robot container and has a joy stick, joystick buttons, swerve subsystem, a sendable chooser for autonomous modes, autonomous modes, and methods for configuring button bindings and smart dashboard options. 
*/

public class RobotContainer {
        /* Controllers */
                public final static Joystick driver = new Joystick(0);
                private final Joystick codriver = new Joystick(1);

        /* Subsystems & Hardware */
                //TODO: add compressor when we have a compressor 
                /* Compressor */
                // private Compressor compressor;
                
                /* Gyro Sensor */
                AHRS ahrs = new AHRS(Port.kMXP); /* Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP or SerialPort.Port.kUSB */
                /* Swerve Subsystem */
                private final Swerve s_Swerve = new Swerve(ahrs);
                private final Intake s_GroundIntake = new Intake();
                private final Shooter s_Shooter = new Shooter(); 
                private final Climb s_Climb = new Climb();
                private final Claw s_Claw = new Claw();
                
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

                /* CoDriver Buttons */
                private final JoystickButton intakeNoteSequenceButton = new JoystickButton(codriver, 
                        XboxController.Button.kRightBumper.value);
                private final JoystickButton intakeNoteButton = new JoystickButton(codriver, 
                        XboxController.Button.kLeftBumper.value);
                private final JoystickButton outtakeButton = new JoystickButton(codriver, 
                XboxController.Button.kA.value);
                private final JoystickButton deployIntakeButton = new JoystickButton(codriver,
                        XboxController.Button.kY.value);
                private final JoystickButton retractIntakeButton = new JoystickButton(codriver, 
                        XboxController.Button.kB.value);
                private final JoystickButton shooterButton = new JoystickButton(codriver, 
                        XboxController.Button.kX.value);
                
                private final JoystickButton openClawButton = new JoystickButton(codriver,XboxController.Button.kLeftStick.value);
                private final JoystickButton closeClawButton = new JoystickButton(codriver,XboxController.Button.kRightStick.value);

        /*Commands */

                private final Command shootCommand = s_Shooter.shootCommand();
                
                private final DeployIntake deployIntake = new DeployIntake(s_GroundIntake);
                private final IntakeNote intakeNote = new IntakeNote(s_GroundIntake);
                private final RetractIntake retractIntake = new RetractIntake(s_GroundIntake);
        
                private final Command outtakeNote = s_GroundIntake.outtakeNoteCommand();
                //THIS IS JUST THE IntakeNote COMMAND AND NOT THE IntakeNoteSequence 
                //private final Command intakeNoteSequence = s_GroundIntake.intakeNoteCommand();
                
                
                
        /* Other */
                /* SendableChooser */
                public final SendableChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                /* Hardware and Logging */
                        DriverStation.silenceJoystickConnectionWarning(true);
                        // TODO: add Camera when we have a camera
                        // CameraServer.startAutomaticCapture();
                        
                        // TODO: add compressor when we have a compressor 
                        // compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);


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

                /* Note Manipulation  */
                //intakeNoteSequenceButton.onTrue(intakeNoteSequence);
                outtakeButton.whileTrue(outtakeNote);
                intakeNoteButton.whileTrue(intakeNote);
                deployIntakeButton.whileTrue(deployIntake);
                retractIntakeButton.whileTrue(retractIntake);
                shooterButton.whileTrue(shootCommand);
        }

        private void configureSmartDashboard() {
                SmartDashboard.putNumber("Swerve/Speed Multiplier", SPEED_MULTIPLIER);
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