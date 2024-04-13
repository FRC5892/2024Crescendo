// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.jni.CANSparkMaxJNI;

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
  private SparkPIDController deployController;
  private DigitalInput deployLimitSwitch;
  private DigitalInput retractLimitSwitch;
  private SparkAbsoluteEncoder deployEncoder;
  double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /* Creates a new GroundIntake. */
  public Intake() {

    intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    deployMotor = new CANSparkMax(IntakeConstants.DEPLOY_MOTOR_ID, MotorType.kBrushless);
    
    deployController = deployMotor.getPIDController();
    deployEncoder = deployMotor.getAbsoluteEncoder(Type.kDutyCycle);
    
    beamBreak = new DigitalInput(IntakeConstants.BEAM_BREAK_DIO_PORT_ID);
    deployLimitSwitch = new DigitalInput(IntakeConstants.DEPLOY_LIMIT_SWITCH_DIO_PORT_ID);
    retractLimitSwitch = new DigitalInput(IntakeConstants.RETRACT_LIMIT_SWITCH_DIO_PORT_ID);
    
    kP = 0.07; 
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1; 
    kMinOutput = -1;

    deployController.setP(kP);
    deployController.setI(kI);
    deployController.setD(kD);
    deployController.setIZone(kIz);
    deployController.setFF(kFF);
    deployController.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

    SmartDashboard.putData("Intake/subsystem", this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/DeployRotations", this.getDeployRotation());
    //SmartDashboard.putNumber("Intake Speed", deployController.calculate(getDeployRotation(), 0.6));
    SmartDashboard.putNumber("Intake/deployIntegrated", deployMotor.getEncoder().getPosition()); 
    SmartDashboard.putBoolean("Intake/deploy", !deployLimitSwitch.get());
    SmartDashboard.putBoolean("Intake/retract", !retractLimitSwitch.get());
    SmartDashboard.putBoolean("Intake/BeamBreak", beamBreak.get());

    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    if((p != kP)) { deployController.setP(p); kP = p; }
    if((i != kI)) { deployController.setI(i); kI = i; }
    if((d != kD)) { deployController.setD(d); kD = d; }
    if((iz != kIz)) { deployController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { deployController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      deployController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    deployController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", deployEncoder.getPosition());
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
      deployController.setReference(0, ControlType.kPosition);
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
        return run(()->this.setDeploySpeed(IntakeConstants.DEPLOY_SPEED))
        // .until(() -> getDeployRotation() <= IntakeConstants.DEPLOYSLOW_ROTATIONS)
        // .andThen(()-> {this.setDeploySpeed(IntakeConstants.DEPLOYSLOW_SPEED); deployMotor.setIdleMode(IdleMode.kBrake);})
        .until(() -> getDeployRotation() <= IntakeConstants.DEPLOY_ROTATIONS||!deployLimitSwitch.get())
        .finallyDo(this::stopDeploy);
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
