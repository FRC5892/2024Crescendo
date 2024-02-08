// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.GroundIntake;

public class DeployIntake extends Command {
  private GroundIntake groundIntake;
  private boolean finish;
  private Timer timer;

  /** Creates a new DeployIntake. */
  public DeployIntake(GroundIntake groundIntake) {
    this.groundIntake = groundIntake;
    finish = false;
    timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(groundIntake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: fix timer
    groundIntake.setDeploySetPoint(IntakeConstants.deploySpeed);

    if (timer.get() > 1) {
      groundIntake.stopDeploy();
      finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
