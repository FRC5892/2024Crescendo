// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.GroundIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeNoteSequence extends SequentialCommandGroup {
  GroundIntake groundIntake;

  /* when i add a lot */
  // IntakeNote intakeNote;
  // DeployIntake deployIntake;
  // RetractIntake retractIntake;

  /* when i don't */
  private final Command intakeNote = groundIntake.intakeNoteCommand();
  private final Command deployIntake = groundIntake.deployIntakeCommand();
  private final Command retractIntake = groundIntake.retractIntakeCommand();

   
  /** Creates a new IntakeNote2. */
  public IntakeNoteSequence() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(intakeNote, deployIntake, retractIntake);
  }
}
