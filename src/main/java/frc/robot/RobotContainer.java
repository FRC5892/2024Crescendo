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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.lib.AutoManager;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.WheelRadiusCharacterization.Direction;
import frc.robot.subsystems.AmpAssist;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve.CenterOfRotation;
import monologue.Monologue;
import monologue.Annotations.Log;
import monologue.Logged;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Logged {
   
        /* Controllers */
                public final static XboxController driver = new XboxController(0);
                public final static XboxController codriver = new XboxController(1);
                public final static CommandXboxController singleDriver = new CommandXboxController(2);

        /* Subsystems & Hardware */
                /* Gyro Sensor */
                AHRS ahrs = Robot.isReal() ? new AHRS(Port.kMXP) : null;

                private final Swerve swerve = new Swerve(ahrs);
                private final Intake intake = new Intake();
                private final Shooter Shooter = new Shooter(); 
                private final Climb climb = new Climb();
                private final Vision vision = new Vision(swerve::useVisionMeasurement,swerve::getPose);
                private final AmpAssist ampAssist = new AmpAssist(); 
                
        /* Controls & Buttons */
                /* Drive Controls */
                private static final int translationAxis = XboxController.Axis.kLeftY.value;
                private static final int strafeAxis = XboxController.Axis.kLeftX.value;
                private static final int rotationAxis = XboxController.Axis.kRightX.value;
                private static double SPEED_MULTIPLIER = 1.0;
                
                /* Driver Buttons */
                private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
                // private final JoystickButton robotCentric = new JoystickButton(driver,
                        // XboxController.Button.kRightBumper.value);
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
                private final Command climbUp = climb.climbUp();
                private final Command climbDown = climb.climbDown();
                @Log private final Command tiltLeft = climb.tiltLeft();
                @Log private final Command tiltRight = climb.tiltRight();
                private final Command followAmpCommand;
        
                /* Codriver  */
                @Log private final Command shootCommand = Shooter.shootCommand();
                private final Command outtakeNote = intake.outtakeNoteCommand();
                private final Command intakeNoteSequence = intake.intakeNoteSequence(driver,codriver);
                private final Command scoreAmpSequence = ampAssist.ampAssistCommand();
                private final Command retractIntake = intake.retractIntakeCommand();
                @Log private final Command deployIntake = intake.deployIntakeCommand();
                
                /* Other */

        /* Other */
                private boolean isSpeedLimited = false;
                @Log Sendable presentationMode = new Sendable() {
                        @Override
                        public void initSendable(SendableBuilder builder) {
                                builder.addBooleanProperty("speedLimit",
                                        ()->isSpeedLimited, 
                                        (b)->{
                                                if (!DriverStation.isFMSAttached()) {isSpeedLimited = b;
                                                        if (isSpeedLimited) {
                                                                SPEED_MULTIPLIER = 0.4;
                                                        } else {
                                                                SPEED_MULTIPLIER = 1;
                                                        }
                                                };
                                });
                            
                        }
                };
                /* SendableChooser */

        public RobotContainer() {
                /* Hardware and Logging */
                        DriverStation.silenceJoystickConnectionWarning(true);
                
                        CameraServer.startAutomaticCapture();

                /* PathPlanner Named Commands */
                        swerve.setupPathPlanner();
                        NamedCommands.registerCommand("deployIntake", intake.deployIntakeCommand());
                        NamedCommands.registerCommand("retractIntake", intake.retractIntakeCommand());
                        NamedCommands.registerCommand("intakeSequence", intake.intakeNoteSequence(driver,codriver));
                        NamedCommands.registerCommand("shootSequence", Shooter.fullShooter(intake));
                        NamedCommands.registerCommand("runShooter", Shooter.shootCommand());
                        NamedCommands.registerCommand("deployAmp", intake.deployAmpCommand());
                        NamedCommands.registerCommand("ampSequence", intake.scoreAmpSequence());
                        NamedCommands.registerCommand("handoffNote", intake.handoffNote());
                        NamedCommands.registerCommand("reducedVisionAmp", vision.reducedDistanceCommand());
                        followAmpCommand = AutoBuilder.buildAuto("Amp Alignment").raceWith(vision.reducedDistanceCommand());
                
                /* Default Commands */
                        swerve.setDefaultCommand(
                                new TeleopSwerve(
                                        swerve,

                                        () -> (-driver.getRawAxis(translationAxis) - singleDriver.getLeftY()) * SPEED_MULTIPLIER,
                                        () -> (-driver.getRawAxis(strafeAxis) - singleDriver.getLeftX()) * SPEED_MULTIPLIER,
                                        () -> -driver.getRawAxis(rotationAxis) - singleDriver.getRightX()* SPEED_MULTIPLIER,
                                        () -> false,
                                        ()-> driver.getRightBumper() ? CenterOfRotation.FRONT_RIGHT : (driver.getLeftBumper() ? CenterOfRotation.FRONT_LEFT : CenterOfRotation.CENTER) ));

                /* Others */
                        AutoManager.useExistingAutoChooser(AutoBuilder.buildAutoChooser());
                        AutoManager.addCharacterization("Wheel Radius", Commands
                                .runOnce(()-> swerve.driveRelative(new ChassisSpeeds(0,0,Units.degreesToRadians(5))), swerve)
                                .andThen(new WaitCommand(0.1))
                                .andThen(()->swerve.stop())
                                .andThen(new WaitCommand(0.15))
                                .andThen(new WheelRadiusCharacterization(swerve, Direction.COUNTER_CLOCKWISE))
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
                zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()).ignoringDisable(true));
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


                singleDriver.y().whileTrue(intake.outtakeNoteCommand());
                singleDriver.x().onTrue(intake.intakeNoteSequence(singleDriver.getHID(), driver));
                singleDriver.a().onTrue(new InstantCommand(() -> swerve.zeroGyro()).ignoringDisable(true));
                singleDriver.b().whileTrue(Shooter.shootCommand());
                singleDriver.leftBumper().whileTrue(intake.deployIntakeCommand());
                singleDriver.rightBumper().whileTrue(intake.retractIntakeCommand());
                singleDriver.povUp().whileTrue(climb.climbUp());
                singleDriver.povDown().whileTrue(climb.climbDown());
        }

        private void configureSmartDashboard() {
                AutoManager.checkForFMS();
                
                this.log("Speed Multiplier", SPEED_MULTIPLIER);
                // turnRight.whileTrue(Commands.runEnd(
                //         ()->{
                //                 Swerve.driveRelative(new ChassisSpeeds(0, 0, -0.5),true);
                //         },
                //         ()->Swerve.stop(),
                //         Swerve));
                // turnLeft.whileTrue(Commands.runEnd(
                // ()->{
                //         Swerve.driveRelative(new ChassisSpeeds(0, 0, 0.5),true);
                // },
                // ()->Swerve.stop(),
                // Swerve));

                // turnRight.onFalse(Swerve.CheckTimeCommand(true));
                // turnLeft.onFalse(Swerve.CheckTimeCommand(false));

                


                Monologue.logObj(new AutoManager(),"Robot/AutoManager");
                Monologue.setupMonologue(this, "Robot",false,true);
        }

        public void disabledInit() {
                swerve.resetToAbsolute();
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