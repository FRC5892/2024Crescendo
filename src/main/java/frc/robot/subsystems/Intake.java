// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.HeroSparkPID;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{

  private CANSparkMax intakeMotor;
  private CANSparkMax deployMotor;
  private DigitalInput beamBreak;
  private HeroSparkPID deployController;
  private DigitalInput deployLimitSwitch;
  private DigitalInput retractLimitSwitch;  

  /* REV’s docs here (https://docs.revrobotics.com/through-bore-encoder/application-examples#ni-roborio) outline the different wiring options:
    If you use through bore encoder as a quadrature / relative encoder, use the Encoder class.
    If you use through bore encoder as a duty cycle / absolute encoder, use the DutyCycleEncoder class.
  If the SparkMax is controlling a brushless motor (NEO/NEO550), you would need to wire it for Alternate Encoder Mode 
    (https://docs.revrobotics.com/sparkmax/operating-modes/using-encoders/alternate-encoder-mode) and use getAlternateEncoder() */
    private SparkAbsoluteEncoder deployEncoder; 

  /* Creates a new GroundIntake. */
  public Intake() {

    intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    deployMotor = new CANSparkMax(IntakeConstants.DEPLOY_MOTOR_ID, MotorType.kBrushless);
    deployEncoder = deployMotor.getAbsoluteEncoder(Type.kDutyCycle);

    
    beamBreak = new DigitalInput(IntakeConstants.BEAM_BREAK_DIO_PORT_ID);
    deployLimitSwitch = new DigitalInput(IntakeConstants.DEPLOY_LIMIT_SWITCH_DIO_PORT_ID);
    retractLimitSwitch = new DigitalInput(IntakeConstants.RETRACT_LIMIT_SWITCH_DIO_PORT_ID);

    deployController = new HeroSparkPID(deployMotor).useAbsoluteEncoder();
    deployController.setPID(IntakeConstants.DEPLOY_PID);
    deployMotor.burnFlash();

    SmartDashboard.putData("Intake/subsystem",this);
    SmartDashboard.putData("Intake/pid",deployController);
  }

  @Override
  public void periodic() {
    // stolen from super


    SmartDashboard.putNumber("Intake/DeployRotations", this.getDeployRotation());
    //SmartDashboard.putNumber("Intake Speed", deployController.calculate(getDeployRotation(), 0.6));
    SmartDashboard.putNumber("Intake/deployIntegrated", deployMotor.getEncoder().getPosition()); 
    SmartDashboard.putNumber("Intake/Setpoint", deployController.getReference());
    SmartDashboard.putBoolean("Intake/deploy", !deployLimitSwitch.get());
    SmartDashboard.putBoolean("Intake/retract", !retractLimitSwitch.get());
    SmartDashboard.putBoolean("Intake/BeamBreak", beamBreak.get());
  }

  /* Other Functions */
    public double getDeployRotation() {
      return deployEncoder.getPosition();
    }

    public void coastMode() {
      deployMotor.setIdleMode(IdleMode.kCoast);
    }

  
  /* Intaking */
    public void intakeNote() {
      intakeMotor.set(Constants.IntakeConstants.INTAKE_SPEED);
    }

    public void outtakeNote() {
      intakeMotor.set(IntakeConstants.OUTTAKE_SPEED);
    }
    public void outtakeNoteForAmp() {
      intakeMotor.set(IntakeConstants.OUTTAKE_SPEED_FOR_AMP);
    }

    //outtakeNoteForAmp

    public void stopIntake() {
      intakeMotor.set(0);
    }


  /* Deploying */  
    public void setDeploySpeed(double speed) {
      deployMotor.set(speed);
    }

    public void stopDeploy() {
      deployMotor.set(0);
      deployController.setReference(0, ControlType.kPosition);

    }

    //PID?
    public void setDeploySetPoint(double setpoint) {
      // deployMotor.set(deployController.calculate(getDeployRotation(), setpoint));
      deployController.setReference(setpoint, ControlType.kPosition);
    }
  


  /* Commands */
    /* Codriver Commands */
      public Command intakeNoteSequence(XboxController controller, XboxController controller2) {
        return deployIntakeCommand()
        .andThen(intakeNoteCommand(controller,controller2))
        .andThen(retractIntakeCommand());
      }

      public Command scoreAmpSequence() {
        return deployAmpCommand()
        .andThen(new WaitCommand(0.5), outtakeNoteForAmpCommand().withTimeout(0.25))
        .andThen(retractIntakeCommand(IntakeConstants.AMP_RETRACT_SPEED).withTimeout(0.1),retractIntakeCommand());
      }

      public Command deployIntakeCommand() {
        // return startEnd(() -> setDeploySetPoint(IntakeConstants.deployRotations), this::stopDeploy).until(() -> deployEncoder.getPosition() <= IntakeConstants.deployRotations ||deployLimitSwitch.get()).andThen(() -> deployMotor.setIdleMode(IdleMode.kCoast));
        return startEnd(()->this.setDeploySpeed(IntakeConstants.DEPLOY_SPEED), this::stopDeploy)
        .until(() -> getDeployRotation() <= IntakeConstants.DEPLOY_ROTATIONS||!deployLimitSwitch.get());
      }

      public Command retractIntakeCommand(double speed) {
        // return startEnd(() -> setDeploySetPoint(IntakeConstants.retractRotations), this::stopDeploy).until(() ->  deployEncoder.getPosition() >= IntakeConstants.retractRotations).andThen(() -> deployMotor.setIdleMode(IdleMode.kBrake));
        return startEnd(()->this.setDeploySpeed(speed), this::stopDeploy)
        .until(() -> getDeployRotation() >= IntakeConstants.RETRACT_ROTATIONS||!retractLimitSwitch.get());
      }
      public Command retractIntakeCommand() {
        return retractIntakeCommand(IntakeConstants.RETRACT_SPEED);
      }


    /* Test Commands */
      public Command intakeNoteCommand(XboxController controller,XboxController controller2) {
        return startEnd(() -> this.intakeNote(), this::stopIntake).until(() -> beamBreak.get()).andThen(() -> {
          controller.setRumble(RumbleType.kBothRumble, 1);
          controller2.setRumble(RumbleType.kBothRumble, 1);
        }).andThen(new WaitCommand(0.25)).andThen(()->{
          controller.setRumble(RumbleType.kBothRumble, 0);
          controller2.setRumble(RumbleType.kBothRumble, 0);
        });
      }

      public Command outtakeNoteCommand() {
        return startEnd(() -> this.outtakeNote(), ()-> this.stopIntake());
      }

      public Command outtakeNoteForAmpCommand() {
        return startEnd(() -> this.outtakeNoteForAmp(), ()-> this.stopIntake());
      }

      public Command deployAmpCommand() {
        return startEnd(() -> this.setDeploySpeed(-0.3), this::stopDeploy).until(() -> getDeployRotation() <= 0.37);
      }

      // public final Command trapezoidCommand = new TrapezoidProfileCommand(
      //   profile,
      //   (state)->setDeploySpeed(state.velocity),
      //   ()->goal,
      //   ()->new State(getDeployRotation(),deployEncoder.getVelocity()),
      //   this);
}
