// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.AmpAssist;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import java.util.function.Function;
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

        private static RobotContainer instance;
        public static RobotContainer getInstance() {
                if (instance == null) instance = new RobotContainer();
                return instance;
        }
        /* Controllers */
                public final static XboxController driver = new XboxController(0);
                public final static XboxController codriver = new XboxController(1);

        /* Subsystems & Hardware */
                /* Gyro Sensor */
                AHRS ahrs = new AHRS(Port.kMXP);

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
                private final POVButton tiltClimbLeftButton = new POVButton(codriver, 270);
                private final POVButton tiltClimbRightButton = new POVButton(codriver, 90);
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
                /* SendableChooser */
                private final SendableChooser<Command> autoChooser;
                private final SendableChooser<Command> characterizationChooser;

        private RobotContainer() {
                /* Hardware and Logging */
                        DriverStation.silenceJoystickConnectionWarning(true);
                        
                        SmartDashboard.putData("IntakeCommand",deployIntake);
                        SmartDashboard.putData("ShootCommand",shootCommand);
                
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
                        SmartDashboard.putBoolean("Characterization", false);
                        autoChooser = AutoBuilder.buildAutoChooser();
                        characterizationChooser = new SendableChooser<Command>();
                        
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
                // tiltClimbLeftButton.whileTrue(tiltLeft);
                // tiltClimbRightButton.whileTrue(tiltRight);

                scoreAmpSequenceButton.toggleOnTrue(scoreAmpSequence);
                deployIntakeButton2.whileTrue(deployIntake);
                retractIntakeButton2.whileTrue(retractIntake);
        }

        private void configureSmartDashboard() {
                SmartDashboard.putNumber("Swerve/Speed Multiplier", SPEED_MULTIPLIER);
                SmartDashboard.putData("Auto Chooser", autoChooser);
                SmartDashboard.putData("tilt-left",tiltLeft);
                SmartDashboard.putData("tilt-right",tiltRight);

        }

        public void disabledInit() {
                s_Swerve.resetToAbsolute();
                driver.setRumble(RumbleType.kBothRumble, 0);
                codriver.setRumble(RumbleType.kBothRumble, 0);
        }
        
        /**
         * Add a Characterization Command to the choosable list on the dashboard 
         * @param name The name of the entry 
         * @param command the command
         */
        public void addCharacterization(String name, Command command) {
                characterizationChooser.addOption(name, command);
        }
        /**
         * Adds all 4 SysId command of a routine to the choosable list on the dashboard.
         * @param name The name of the routine, seen on the dashboard 
         * @param map Function to map each command to. Could be used to add before and after commands. 
         * @param routine The SysID routine
         */
        public void addSysidCharacterization(String name, Function<Command,Command> map, SysIdRoutine routine) {
                addCharacterization(name+" Dynamic Forward", map.apply(routine.dynamic(Direction.kForward)));
                addCharacterization(name+" Dynamic Reverse", map.apply(routine.dynamic(Direction.kReverse)));
                addCharacterization(name+" Quasistatic Forward", map.apply(routine.quasistatic(Direction.kForward)));
                addCharacterization(name+" Quasistatic Reverse", map.apply(routine.quasistatic(Direction.kReverse)));
        }
        /**
         * Adds all 4 SysId command of a routine to the choosable list on the dashboard.
         * @param name The name of the routine, seen on the dashboard 
         * @param routine The SysID routine
         */
        public void addSysidCharacterization(String name, SysIdRoutine routine) { 
                addSysidCharacterization(name, c-> c, routine);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                if (SmartDashboard.getBoolean("Characterization",false)) {
                        return characterizationChooser.getSelected(); 
                } else {
                        return autoChooser.getSelected();
                }
        }

        
}