// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AddVisionPose extends Command{
  private Vision vision;
  private Swerve swerveSubsytem;
 /** Creates a new AprilTagLocation. */
  public AddVisionPose(Vision vision3, Swerve swerve) {
    this.vision = vision3;
    this.swerveSubsytem = swerve;
    addRequirements(vision);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Pose2d estimatePose = swerveSubsytem.addVisionMeasurement(vision.getVisionPose(), vision.getVisionTimestamp());
    vision.setReferencePose(estimatePose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  @Override
  public boolean runsWhenDisabled() {
        return true;
  }
}