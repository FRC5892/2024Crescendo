// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.GroundIntake;

public class IntakeNote extends Command {
  private GroundIntake groundIntake;
  private boolean finish;
  private DigitalInput beamBreak;

  /** Creates a new IntakeNote. */
  public IntakeNote(GroundIntake groundIntake) {
    this.groundIntake = groundIntake;
    beamBreak = new DigitalInput(Constants.IntakeConstants.beamBreakPort);
    finish = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(groundIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if the beam break is tripped, stop intaking
    if (beamBreak.get()) {
      groundIntake.stopIntake();
      finish = true;
    } else {
      groundIntake.runIntake();;
    }
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
