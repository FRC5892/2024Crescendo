// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.GroundIntake;

public class IntakeNote extends Command {
  GroundIntake groundIntake;
  public static Timer timer;

  /** Creates a new IntakeNote. */
  public IntakeNote(GroundIntake groundIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.groundIntake = groundIntake;
    addRequirements(groundIntake);
    timer = new Timer();
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
    //TODO: get the right rotations for deploy intake/intake note commands
    groundIntake.deployIntake();

    if (timer.get() > 1) {
      groundIntake.intakeNote();
    } else if (timer.get() > 2) {
      groundIntake.retractIntake();
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
    return timer.get() > 3;
  }
}
