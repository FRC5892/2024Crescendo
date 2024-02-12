// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class DeployIntake extends Command {
  private Intake groundIntake;
  private boolean finish;
  private DigitalInput limitSwitch;

  /** Creates a new DeployIntake. */
  public DeployIntake(Intake groundIntake) {
    this.groundIntake = groundIntake;
    finish = false;
    limitSwitch = new DigitalInput(IntakeConstants.deployLimitSwitchPort);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(groundIntake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("hi2");
    
    SmartDashboard.putNumber("hi2", groundIntake.getDeployRotation());

    if (groundIntake.getDeployRotation() >= 0.5) {
      SmartDashboard.putBoolean("hi", true);
      groundIntake.stopDeploy();
      finish = true;
    } else {
      SmartDashboard.putBoolean("hi", false);
      System.out.println("deployintakefromcommand");

      groundIntake.deployIntake();
    }

    //if the beam break is tripped, stop retract
    // if (!limitSwitch.get()) {
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
