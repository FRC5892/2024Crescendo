// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.HeroSparkPID;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import monologue.Logged;
import monologue.Annotations.Log;

public class Intake extends SubsystemBase implements Logged{

  private CANSparkMax intakeMotor;
  private CANSparkMax deployMotor;
  private DigitalInput beamBreak;
  @Log private HeroSparkPID deployController;
  private DigitalInput deployLimitSwitch;
  private DigitalInput retractLimitSwitch;
  private SparkAbsoluteEncoder deployEncoder;
  double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /* Creates a new GroundIntake. */
  public Intake() {

    intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    deployMotor = new CANSparkMax(IntakeConstants.DEPLOY_MOTOR_ID, MotorType.kBrushless);
    deployController = new HeroSparkPID(deployMotor);
    deployEncoder = deployMotor.getAbsoluteEncoder(Type.kDutyCycle);
    
    beamBreak = new DigitalInput(IntakeConstants.BEAM_BREAK_DIO_PORT_ID);
    deployLimitSwitch = new DigitalInput(IntakeConstants.DEPLOY_LIMIT_SWITCH_DIO_PORT_ID);
    retractLimitSwitch = new DigitalInput(IntakeConstants.RETRACT_LIMIT_SWITCH_DIO_PORT_ID);
  }

  @Override
  public void periodic() {
    this.log("DeployRotations", this.getDeployRotation());
    //this.log("Intake Speed", deployController.calculate(getDeployRotation(), 0.6));
    this.log("deployIntegrated", deployMotor.getEncoder().getPosition()); 
    this.log("deploy", !deployLimitSwitch.get());
    this.log("retract", !retractLimitSwitch.get());
    this.log("BeamBreak", beamBreak.get());
    this.log("reference",  deployController.getReference());
  }

  /**
   * @return the position of the intake deploy motor
   */
  public double getDeployRotation() {
    return deployEncoder.getPosition();
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
    public void stopIntake() {
      intakeMotor.set(0);
    }

  /* Deploying */  
    public void setDeploySpeed(double speed) {
      deployMotor.set(speed);
    }

    public void stopDeploy() {
      deployMotor.set(0);
    }

    //PID?
    public void setDeploySetpoint(double setpoint) {
      deployController.setReference(setpoint, ControlType.kPosition);
    }

    public Command setTestSetpointCommand() {
      return runOnce(() -> this.setDeploySetpoint(0.3));
    }
    
    /* Commands */

    /**
     * Runs the intake rollers to outtake until interrupted.
     */
    public Command outtakeNoteCommand() {
      return startEnd(() -> this.outtakeNote(), ()-> this.stopIntake());
    }

    /**
     * Runs the intake rollers to intake a note until interrupted, or until the intake beam break is triggered.
     * Rumbles the driver and codriver controllers to let the operators know that a note is now in the intake.
     * @param controller
     * @param controller2
     */
    public Command intakeNoteCommand(XboxController controller, XboxController controller2) {
      return startEnd(() -> this.intakeNote(), this::stopIntake).until(() -> beamBreak.get())
      .andThen(() -> {
        controller.setRumble(RumbleType.kBothRumble, 1);
        controller2.setRumble(RumbleType.kBothRumble, 1);
      })
      .andThen(new WaitCommand(0.25))
      .finallyDo(()->{
        controller.setRumble(RumbleType.kBothRumble, 0);
        controller2.setRumble(RumbleType.kBothRumble, 0);
      });
    }

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
        // return run(()->this.setDeploySpeed(IntakeConstants.DEPLOY_SPEED))
        return run(()->this.setDeploySetpoint(IntakeConstants.DEPLOY_VALUE))
        // .until(() -> getDeployRotation() <= IntakeConstants.DEPLOYSLOW_ROTATIONS)
        // .andThen(()-> {this.setDeploySpeed(IntakeConstants.DEPLOYSLOW_SPEED); deployMotor.setIdleMode(IdleMode.kBrake);})
        .until(() -> !deployLimitSwitch.get())
        .finallyDo(this::stopDeploy);
      }
      public Command retractIntakeCommand(double speed) {
        // return startEnd(() -> setDeploySetPoint(IntakeConstants.retractRotations), this::stopDeploy).until(() ->  deployEncoder.getPosition() >= IntakeConstants.retractRotations).andThen(() -> deployMotor.setIdleMode(IdleMode.kBrake));
        // return startEnd(()->this.setDeploySpeed(speed), this::stopDeploy)
        return startEnd(()->this.setDeploySetpoint(IntakeConstants.RETRACT_VALUE), this::stopDeploy)
        .until(() -> getDeployRotation() >= IntakeConstants.RETRACT_VALUE||!retractLimitSwitch.get());
      }
      public Command retractIntakeCommand() {
        return retractIntakeCommand(IntakeConstants.RETRACT_SPEED);
      }


    /* Test Commands */

    public Command handoffNote() {
      return outtakeNoteCommand()
      .withTimeout(0.5);
    }

    public Command outtakeNoteForAmpCommand() {
      return startEnd(() -> this.outtakeNoteForAmp(), ()-> this.stopIntake());
    }
    
    public Command deployAmpCommand() {
      return startEnd(() -> this.setDeploySpeed(-0.3), this::stopDeploy).until(() -> getDeployRotation() <= 0.37);
    }
}
