// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class RetractIntake extends Command {
  private Intake groundIntake;
  private boolean finish;

  /** Creates a new DeployIntake. */
  public RetractIntake(Intake groundIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.groundIntake = groundIntake;
    //limitSwitch = new DigitalInput(IntakeConstants.retractLimitSwitchPort);
    addRequirements(groundIntake);
    finish = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double encoderPosition = groundIntake.getDeployRotation();

    groundIntake.retractIntake();

    if (encoderPosition <= 0.2){
      //TODO: switch with PID
      groundIntake.stopDeploy();
      finish = true;
    } 

    //if the beam break is tripped, stop retract
    // if (limitSwitch.get()) {
    //   groundIntake.stopDeploy();
    //   finish = true;
    // } 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}