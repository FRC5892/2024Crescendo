// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntake;

public class IntakeGamepiece extends Command {

  private GroundIntake groundIntake;
  private boolean finish;

  /** Creates a new IntakeGamepiece. */
  public IntakeGamepiece(GroundIntake groundIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.groundIntake = groundIntake;
    finish = false;

    addRequirements(groundIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    groundIntake.runIntake();
    finish = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    groundIntake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
