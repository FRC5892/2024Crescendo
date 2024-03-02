// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
// import frc.robot.autos.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Climb.Claw;
import frc.robot.subsystems.Climb.Climb;

/* 
Summary:
This code is for the robot container and has a joy stick, joystick buttons, swerve subsystem, a sendable chooser for autonomous modes, autonomous modes, and methods for configuring button bindings and smart dashboard options. 
*/

public class RobotContainer {
        /* Controllers */
                public final static XboxController driver = new XboxController(0);
                public final static XboxController codriver = new XboxController(1);
                public final static XboxController testDriver = new XboxController(2);

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
                private final Vision s_Vision = new Vision();
                
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

                /* Co-Driver Buttons */
                private final JoystickButton intakeNoteSequenceButton = new JoystickButton(codriver, 
                        XboxController.Button.kX.value);
                private final JoystickButton shootButton = new JoystickButton(codriver, 
                        XboxController.Button.kY.value);
                private final JoystickButton revShooterButton = new JoystickButton(codriver,
                        XboxController.Button.kB.value);
                private final JoystickButton scoreAmpSequenceButton = new JoystickButton(codriver, 
                        XboxController.Button.kA.value);   
                private final POVButton climbUpButton = new POVButton(codriver, 0);
                private final POVButton climbDownButton = new POVButton(codriver, 180);
                private final POVButton tiltClimbLeftButton = new POVButton(codriver, 270);
                private final POVButton tiltClimbRightButton = new POVButton(codriver, 90);
                private final JoystickButton deployIntakeButton2 = new JoystickButton(codriver,
                        XboxController.Button.kLeftBumper.value);
                private final JoystickButton retractIntakeButton2 = new JoystickButton(codriver,
                        XboxController.Button.kRightBumper.value);
                
                        
                /* Test-Driver buttons */
                private final JoystickButton intakeNoteButton = new JoystickButton(testDriver, 
                        XboxController.Button.kX.value);
                private final JoystickButton outtakeButton = new JoystickButton(testDriver, 
                        XboxController.Button.kA.value);
                private final JoystickButton climbLeftButton = new JoystickButton(testDriver,
                        XboxController.Button.kY.value);
                private final JoystickButton retractIntakeButton = new JoystickButton(testDriver, 
                        XboxController.Button.kB.value);
                private final JoystickButton testClimbUpButton = new JoystickButton(testDriver, 
                        XboxController.Button.kLeftBumper.value);
                private final JoystickButton testClimbDownButton = new JoystickButton(testDriver, 
                        XboxController.Button.kRightBumper.value);
                // private final JoystickButton intakeClawButton = new JoystickButton(codriver,
                //         XboxController.Button.kLeftStick.value);
                // private final JoystickButton outtakeClawButton = new JoystickButton(codriver,
                //         XboxController.Button.kRightStick.value);
        
        /* Commands */
                /* Driver */
                private final Command climbUp = s_Climb.climbUp();
                private final Command climbDown = s_Climb.climbDown();
                private final Command tiltLeft = s_Climb.tiltLeft();
                private final Command tiltRight = s_Climb.tiltRight();

                /* Test */
                private final Command intakeNote = s_GroundIntake.intakeNoteCommand();
                private final Command climbLeft = s_Climb.climbLeftDown();
                // private final Command openClawCommand = s_Claw.openClawCommand();
                // private final Command closeClawCommand = s_Claw.closeClawCommand();
                //private final Command fullShootCommand = s_Shooter.fullShooter(s_GroundIntake);
        
                /* Codriver  */
                private final Command shootCommand = s_Shooter.shootCommand();
                private final Command outtakeNote = s_GroundIntake.outtakeNoteCommand();
                private final Command intakeNoteSequence = s_GroundIntake.intakeNoteSequence();
                private final Command scoreAmpSequence = s_GroundIntake.scoreAmpSequence();
                private final Command retractIntake = s_GroundIntake.retractIntakeCommand();
                private final Command deployIntake = s_GroundIntake.deployIntakeCommand();
                
                /* General */
                private final Command addVisionPose = new AddVisionPose(s_Vision,s_Swerve); 
                
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
                        SmartDashboard.putData("IntakeCommand",deployIntake);
                        SmartDashboard.putData("ShootCommand",shootCommand);
                
                /* PathPlanner Named Commands */
                        s_Swerve.setupPathPlanner();
                        NamedCommands.registerCommand("deployIntake", s_GroundIntake.deployIntakeCommand());
                        NamedCommands.registerCommand("retractIntake", s_GroundIntake.retractIntakeCommand());
                        NamedCommands.registerCommand("intakeSequence", s_GroundIntake.intakeNoteSequence());
                        NamedCommands.registerCommand("shootSequence", s_Shooter.fullShooter(s_GroundIntake));
                        NamedCommands.registerCommand("deployAmp", s_GroundIntake.deployAmpCommand());
                        NamedCommands.registerCommand("ampSequence", s_GroundIntake.scoreAmpSequence());

                /* Default Commands */
                        s_Swerve.setDefaultCommand(
                                new TeleopSwerve(
                                        s_Swerve,

                                        () -> -driver.getRawAxis(translationAxis) * SPEED_MULTIPLIER,
                                        () -> -driver.getRawAxis(strafeAxis) * SPEED_MULTIPLIER,
                                        () -> -driver.getRawAxis(rotationAxis) * SPEED_MULTIPLIER,
                                        () -> robotCentric.getAsBoolean()));
                        s_Vision.setDefaultCommand(addVisionPose);

                /* Others */
                        // Auto chooser
                        autoChooser = AutoBuilder.buildAutoChooser("Center 2 note auto");
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
                
                
                /* Testing */
                outtakeButton.whileTrue(outtakeNote);
                intakeNoteButton.whileTrue(intakeNote);
                climbLeftButton.whileTrue(climbLeft);
                retractIntakeButton.whileTrue(retractIntake);
                
                
                /* Codriver Buttons */
                intakeNoteSequenceButton.onTrue(intakeNoteSequence);
                revShooterButton.whileTrue(shootCommand);
                shootButton.whileTrue(outtakeNote);
                climbUpButton.whileTrue(climbUp);
                climbDownButton.whileTrue(climbDown);
                tiltClimbLeftButton.whileTrue(tiltLeft);
                tiltClimbRightButton.whileTrue(tiltRight);

                scoreAmpSequenceButton.whileTrue(scoreAmpSequence);
                deployIntakeButton2.whileTrue(deployIntake);
                retractIntakeButton2.whileTrue(retractIntake);

                // intakeClawButton.whileTrue(openClawCommand);
                // outtakeClawButton.whileTrue(closeClawCommand);
                testClimbUpButton.whileTrue(climbUp);
                testClimbDownButton.whileTrue(climbDown);
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