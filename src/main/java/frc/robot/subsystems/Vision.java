// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  public float target;
  private PhotonCamera camera;
  private PhotonPoseEstimator poseEstimator;
  private AprilTagFieldLayout fieldLayout;
  private double poseTimestamp;
  private Pose2d visionPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  private Field2d field2d;
  private Pose2d referencePose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

  /**
   * Creates a new AprilTagVision3.
   * 
   * @throws IOException
   */
  public Vision() {

    camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
    try {
      fieldLayout = AprilTagFieldLayout.loadFromResource(VisionConstants.FIELD_LAYOUT_RESOURCE_FILE);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
    poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera,
        VisionConstants.ROBOT_TO_CAM);
    field2d = new Field2d();
    SmartDashboard.putData("Vision estimated Pose",field2d);

    poseTimestamp = Timer.getFPGATimestamp();
  }
  
  // feed into SwerveDrivePoseEstimator
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    poseEstimator.setReferencePose(prevEstimatedRobotPose);
    return poseEstimator.update();
    
  }
  public Pose2d getVisionPose() {
    return visionPose;
  } 
  public Double getVisionTimestamp () {
    return poseTimestamp;
  }
  
  public Pose2d getReferencePose() {
    return referencePose;
  }
  public void setReferencePose(Pose2d referencePose) {
    this.referencePose = referencePose;
  } 
  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose(referencePose);
    if (estimatedPose.isPresent()) {
      this.visionPose = estimatedPose.get().estimatedPose.toPose2d();
      this.poseTimestamp = estimatedPose.get().timestampSeconds;
    }
    var result = camera.getLatestResult();
    field2d.setRobotPose(this.visionPose);
    
    


    
    SmartDashboard.putNumber("Vision estimated Angle",getVisionPose().getRotation().getDegrees());
    SmartDashboard.putBoolean("Has Targets", result.hasTargets());

  }
  
}