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
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.WheelRadiusCharacterization.Direction;
import frc.robot.subsystems.AmpAssist;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
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

        /* Subsystems & Hardware */
                /* Gyro Sensor */
                AHRS ahrs = Robot.isReal() ? new AHRS(Port.kMXP) : null;

                private final Swerve Swerve = new Swerve(ahrs);
                private final Intake Intake = new Intake();
                private final Shooter Shooter = new Shooter(); 
                private final Climb Climb = new Climb();
                private final Vision Vision = new Vision(Swerve::useVisionMeasurement,Swerve::getPose);
                private final AmpAssist AmpAssist = new AmpAssist(); 
                
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
                private final JoystickButton turnLeft = new JoystickButton(driver, XboxController.Button.kX.value);
                private final JoystickButton turnRight = new JoystickButton(driver, XboxController.Button.kB.value);


        
        /* Commands */
                /* Driver */
                private final Command climbUp = Climb.climbUp();
                private final Command climbDown = Climb.climbDown();
                @Log private final Command tiltLeft = Climb.tiltLeft();
                @Log private final Command tiltRight = Climb.tiltRight();
                private final Command followAmpCommand;
        
                /* Codriver  */
                @Log private final Command shootCommand = Shooter.shootCommand();
                private final Command outtakeNote = Intake.outtakeNoteCommand();
                private final Command intakeNoteSequence = Intake.intakeNoteSequence(driver,codriver);
                private final Command scoreAmpSequence = AmpAssist.ampAssistCommand();
                private final Command retractIntake = Intake.retractIntakeCommand();
                @Log private final Command deployIntake = Intake.deployIntakeCommand();
                
                /* Other */

        /* Other */
                /* SendableChooser */

        public RobotContainer() {
                /* Hardware and Logging */
                        DriverStation.silenceJoystickConnectionWarning(true);
                
                        CameraServer.startAutomaticCapture();

                /* PathPlanner Named Commands */
                        Swerve.setupPathPlanner();
                        NamedCommands.registerCommand("deployIntake", Intake.deployIntakeCommand());
                        NamedCommands.registerCommand("retractIntake", Intake.retractIntakeCommand());
                        NamedCommands.registerCommand("intakeSequence", Intake.intakeNoteSequence(driver,codriver));
                        NamedCommands.registerCommand("shootSequence", Shooter.fullShooter(Intake));
                        NamedCommands.registerCommand("runShooter", Shooter.shootCommand());
                        NamedCommands.registerCommand("deployAmp", Intake.deployAmpCommand());
                        NamedCommands.registerCommand("ampSequence", Intake.scoreAmpSequence());
                        NamedCommands.registerCommand("handoffNote", Intake.handoffNote());
                        NamedCommands.registerCommand("reducedVisionAmp", Vision.reducedDistanceCommand());
                        followAmpCommand = AutoBuilder.buildAuto("Amp Alignment").raceWith(Vision.reducedDistanceCommand());
                
                /* Default Commands */
                        Swerve.setDefaultCommand(
                                new TeleopSwerve(
                                        Swerve,

                                        () -> -driver.getRawAxis(translationAxis) * SPEED_MULTIPLIER,
                                        () -> -driver.getRawAxis(strafeAxis) * SPEED_MULTIPLIER,
                                        () -> -driver.getRawAxis(rotationAxis) * SPEED_MULTIPLIER,
                                        () -> robotCentric.getAsBoolean()));

                /* Others */
                        AutoManager.useExistingAutoChooser(AutoBuilder.buildAutoChooser());
                        AutoManager.addCharacterization("Wheel Radius", Commands
                                .runOnce(()-> Swerve.driveRelative(new ChassisSpeeds(0,0,Units.degreesToRadians(5))), Swerve)
                                .andThen(new WaitCommand(0.1))
                                .andThen(()->Swerve.stop())
                                .andThen(new WaitCommand(0.15))
                                .andThen(new WheelRadiusCharacterization(Swerve, Direction.COUNTER_CLOCKWISE))
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
                zeroGyro.onTrue(new InstantCommand(() -> Swerve.zeroGyro()).ignoringDisable(true));
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
                AutoManager.checkForFMS();
                
                this.log("Speed Multiplier", SPEED_MULTIPLIER);
                turnRight.whileTrue(Commands.runEnd(
                        ()->{
                                Swerve.driveRelative(new ChassisSpeeds(0, 0, -0.5),true);
                        },
                        ()->Swerve.stop(),
                        Swerve));
                turnLeft.whileTrue(Commands.runEnd(
                ()->{
                        Swerve.driveRelative(new ChassisSpeeds(0, 0, 0.5),true);
                },
                ()->Swerve.stop(),
                Swerve));

                turnRight.onFalse(Swerve.CheckTimeCommand(true));
                turnLeft.onFalse(Swerve.CheckTimeCommand(false));

                


                Monologue.logObj(new AutoManager(),"Robot/AutoManager");
                Monologue.setupMonologue(this, "Robot",false,true);
        }

        public void disabledInit() {
                Swerve.resetToAbsolute();
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