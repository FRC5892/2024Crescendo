// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeNoteSequence extends SequentialCommandGroup {
  /* when i add a lot */
  private IntakeNote intakeNote;
  private DeployIntake deployIntake;
  private RetractIntake retractIntake;

   
  /** Creates a new IntakeNote2. */
  public IntakeNoteSequence() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(deployIntake, intakeNote, retractIntake);
  }
}
