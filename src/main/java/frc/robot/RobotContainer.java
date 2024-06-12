// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.lib.AutoManager;
import frc.lib.HeroLogger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.WheelRadiusCharacterization.Direction;
import frc.robot.subsystems.AmpAssist;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        private static HeroLogger logger = new HeroLogger("Robot Container");
   
        /* Controllers */
                public final static XboxController driver = new XboxController(0);
                public final static XboxController codriver = new XboxController(1);

        /* Subsystems & Hardware */
                /* Gyro Sensor */
                AHRS ahrs = Robot.isReal() ? new AHRS(Port.kMXP) : null;

                /* Swerve Subsystem */
                private final Swerve s_Swerve = new Swerve(ahrs);
                private final Intake s_GroundIntake = new Intake();
                private final Shooter s_Shooter = new Shooter(); 
                private final Climb s_Climb = new Climb();
                private final Vision s_Vision = new Vision(s_Swerve::useVisionMeasurement,s_Swerve::getPose);
                private final AmpAssist s_AmpAssist = new AmpAssist(); 
                
        /* Controls & Buttons */
                /* Drive Controls */
                private static final int translationAxis = XboxController.Axis.kLeftY.value;
                private static final int strafeAxis = XboxController.Axis.kLeftX.value;
                private static final int rotationAxis = XboxController.Axis.kRightX.value;
                private static final double SPEED_MULTIPLIER = 1.0;
                
                /* Driver Buttons */
                private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
                private final JoystickButton robotCentric = new JoystickButton(driver,
                        XboxController.Button.kRightBumper.value);
                private final JoystickButton alignAmpButton = new JoystickButton(driver, XboxController.Button.kA.value);

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
                private final JoystickButton deployIntakeButton2 = new JoystickButton(codriver,
                        XboxController.Button.kLeftBumper.value);
                private final JoystickButton retractIntakeButton2 = new JoystickButton(codriver,
                        XboxController.Button.kRightBumper.value);
        
        /* Commands */
                /* Driver */
                private final Command climbUp = s_Climb.climbUp();
                private final Command climbDown = s_Climb.climbDown();
                private final Command tiltLeft = s_Climb.tiltLeft();
                private final Command tiltRight = s_Climb.tiltRight();
                private final Command followAmpCommand;
        
                /* Codriver  */
                private final Command shootCommand = s_Shooter.shootCommand();
                private final Command outtakeNote = s_GroundIntake.outtakeNoteCommand();
                private final Command intakeNoteSequence = s_GroundIntake.intakeNoteSequence(driver,codriver);
                private final Command scoreAmpSequence = s_AmpAssist.ampAssistCommand();
                private final Command retractIntake = s_GroundIntake.retractIntakeCommand();
                private final Command deployIntake = s_GroundIntake.deployIntakeCommand();
                
                /* Other */

        /* Other */
                /* SendableChooser */

        public RobotContainer() {
                /* Hardware and Logging */
                        DriverStation.silenceJoystickConnectionWarning(true);
                        
                        logger.log("IntakeCommand",deployIntake);
                        logger.log("ShootCommand",shootCommand);
                
                        CameraServer.startAutomaticCapture();

                /* PathPlanner Named Commands */
                        s_Swerve.setupPathPlanner();
                        NamedCommands.registerCommand("deployIntake", s_GroundIntake.deployIntakeCommand());
                        NamedCommands.registerCommand("retractIntake", s_GroundIntake.retractIntakeCommand());
                        NamedCommands.registerCommand("intakeSequence", s_GroundIntake.intakeNoteSequence(driver,codriver));
                        NamedCommands.registerCommand("shootSequence", s_Shooter.fullShooter(s_GroundIntake));
                        NamedCommands.registerCommand("runShooter", s_Shooter.shootCommand());
                        NamedCommands.registerCommand("deployAmp", s_GroundIntake.deployAmpCommand());
                        NamedCommands.registerCommand("ampSequence", s_GroundIntake.scoreAmpSequence());
                        NamedCommands.registerCommand("handoffNote", s_GroundIntake.handoffNote());
                        NamedCommands.registerCommand("reducedVisionAmp", s_Vision.reducedDistanceCommand());
                        followAmpCommand = AutoBuilder.buildAuto("Amp Alignment").raceWith(s_Vision.reducedDistanceCommand());
                
                /* Default Commands */
                        s_Swerve.setDefaultCommand(
                                new TeleopSwerve(
                                        s_Swerve,

                                        () -> -driver.getRawAxis(translationAxis) * SPEED_MULTIPLIER,
                                        () -> -driver.getRawAxis(strafeAxis) * SPEED_MULTIPLIER,
                                        () -> -driver.getRawAxis(rotationAxis) * SPEED_MULTIPLIER,
                                        () -> robotCentric.getAsBoolean()));

                /* Others */
                        AutoManager.useExistingAutoChooser(AutoBuilder.buildAutoChooser());
                        AutoManager.addCharacterization("Wheel Radius", Commands
                                .runOnce(()-> s_Swerve.driveRelative(new ChassisSpeeds(0,0,Units.degreesToRadians(5))), s_Swerve)
                                .andThen(new WaitCommand(0.1))
                                .andThen(()->s_Swerve.stop())
                                .andThen(new WaitCommand(0.15))
                                .andThen(new WheelRadiusCharacterization(s_Swerve, Direction.COUNTER_CLOCKWISE))
                        );
                        
                        configureButtonBindings();
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
                zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()).ignoringDisable(true));
                alignAmpButton.whileTrue(followAmpCommand);
                /* Codriver Buttons */
                intakeNoteSequenceButton.onTrue(intakeNoteSequence);
                revShooterButton.whileTrue(shootCommand);
                shootButton.whileTrue(outtakeNote);
                climbUpButton.whileTrue(climbUp);
                climbDownButton.whileTrue(climbDown);

                scoreAmpSequenceButton.toggleOnTrue(scoreAmpSequence);
                deployIntakeButton2.whileTrue(deployIntake);
                retractIntakeButton2.whileTrue(retractIntake);
        }

        private void configureSmartDashboard() {
                HeroLogger.getDashboard().log("Speed Multiplier", SPEED_MULTIPLIER);
                logger.log("tilt-left",tiltLeft);
                logger.log("tilt-right",tiltRight);
                logger.log("Test_command", Commands.runEnd(()->s_Swerve.driveRelative(new ChassisSpeeds(0, 0, 0.25),true), ()->s_Swerve.stop(), s_Swerve));

                AutoManager.initDashboard();
        }

        public void disabledInit() {
                s_Swerve.resetToAbsolute();
                driver.setRumble(RumbleType.kBothRumble, 0);
                codriver.setRumble(RumbleType.kBothRumble, 0);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return AutoManager.getSelected();
        }

        
}