// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.HeroLogger;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import monologue.Logged;
import monologue.Annotations.Log;

public class Vision extends SubsystemBase implements Logged{
  private PhotonCamera frontCamera; private PhotonCamera backCamera;

  private PhotonPoseEstimator frontEstimator; private PhotonPoseEstimator backEstimator;

  private AprilTagFieldLayout fieldLayout;
  private double poseTimestamp;
  private Pose2d visionPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  @Log private Field2d field2d;
  private Pose2d referencePose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  private Consumer<VisionMeasurement> consumer;
  private Supplier<Pose2d> poseSupplier;
  private double noisyDistanceMeters = VisionConstants.NOISY_DISTANCE_METERS;
  public static class VisionMeasurement {
    public Pose2d pose;
    public double timeStamp;
    // Matrix<N3, N1> dev;
    // public VisionMeasurement (Pose2d pose,double timeStamp,Matrix<N3, N1> dev) {
    public VisionMeasurement (Pose2d pose,double timeStamp) {
      this.pose = pose;
      // this.dev = dev;
      this.timeStamp = timeStamp;
    }
  }
  public Vision(Consumer<VisionMeasurement> consumer,Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
    this.consumer = consumer;
    frontCamera = new PhotonCamera(VisionConstants.FRONT_CAMERA_NAME);
    backCamera = new PhotonCamera(VisionConstants.BACK_CAMERA_NAME);
    
    field2d = new Field2d();

    try {
      fieldLayout = AprilTagFieldLayout.loadFromResource(VisionConstants.FIELD_LAYOUT_RESOURCE_FILE);
    } catch (IOException e) {throw new UncheckedIOException(e);}
    frontEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, frontCamera,
        VisionConstants.ROBOT_TO_FRONT_CAM);
    backEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, backCamera,
        VisionConstants.ROBOT_TO_BACK_CAM);

    poseTimestamp = Timer.getFPGATimestamp();
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
  public void setNoisyDistance(double distanceMeters) {
    noisyDistanceMeters = distanceMeters;
  }
  public void resetNoisyDistance() {
    setNoisyDistance(VisionConstants.NOISY_DISTANCE_METERS); 
  }
 public Command reducedDistanceCommand() {
  return startEnd(()->setNoisyDistance(VisionConstants.AMP_NOISY_DISTANCE_METERS), this::resetNoisyDistance);
 }


  @Override
  public void periodic() {
    /* update estimated pose */
    referencePose = poseSupplier.get();
    frontEstimator.setReferencePose(referencePose);
    backEstimator.setReferencePose(referencePose);

    Optional<EstimatedRobotPose> frontEstimate = frontEstimator.update();
    Optional<EstimatedRobotPose> backEstimate  = backEstimator.update();
    Pose3d[] frontTags; 
    if (frontEstimate.isPresent()) {
      // consumer.accept(new VisionMeasurement(frontEstimate.get().estimatedPose.toPose2d(), frontEstimate.get().timestampSeconds, confidenceCalculator(frontEstimate.get())));
      if (confidenceCalculator(frontEstimate.get())) {
        consumer.accept(new VisionMeasurement(frontEstimate.get().estimatedPose.toPose2d(), frontEstimate.get().timestampSeconds));
        this.visionPose = frontEstimate.get().estimatedPose.toPose2d();

      }
        //good old java
        frontTags = frontEstimate.get().targetsUsed.stream()
          .map((i)-> fieldLayout.getTagPose(i.getFiducialId()).get())
          .toArray(size -> new Pose3d[size]);
       
      
    } else {
      frontTags = new Pose3d[0];
    }
    
    Pose3d[] backTags; 
    if (backEstimate.isPresent()) {
      // consumer.accept(new VisionMeasurement(backEstimate.get().estimatedPose.toPose2d(), backEstimate.get().timestampSeconds, confidenceCalculator(backEstimate.get())));
      if (confidenceCalculator(backEstimate.get())) {
        consumer.accept(new VisionMeasurement(backEstimate.get().estimatedPose.toPose2d(), backEstimate.get().timestampSeconds));
        this.visionPose = backEstimate.get().estimatedPose.toPose2d();
      }
      //good old java
      backTags= backEstimate.get().targetsUsed.stream()
      .map((i)-> fieldLayout.getTagPose(i.getFiducialId()).get())
      .toArray(size -> new Pose3d[size]);
      
    } else {
      backTags= new Pose3d[0];
    }
    if (!frontCamera.isConnected()) frontTags = new Pose3d[0];
    if (!backCamera.isConnected()) backTags = new Pose3d[0];
    

    this.log("Front Tags",frontTags);
    this.log("Back Tags",backTags);
    
    this.log("Front Camera Connected", frontCamera.isConnected());
    this.log("Back Camera Connected", backCamera.isConnected());
    field2d.setRobotPose(this.visionPose);
    
    this.log("Estimated Angle",getVisionPose().getRotation().getDegrees());
  }
  // private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
   private boolean confidenceCalculator(EstimatedRobotPose estimation) {
    double smallestDistance = Double.POSITIVE_INFINITY;
    for (var target : estimation.targetsUsed) {
      var t3d = target.getBestCameraToTarget();
      var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
      if (distance < smallestDistance)
        smallestDistance = distance;
    }

    // double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
    //     ? 1
    //     : Math.max(
    //         1,
    //         (estimation.targetsUsed.get(0).getPoseAmbiguity()
    //             + Constants.VisionConstants.POSE_AMBIGUITY_SHIFTER)
    //             * Constants.VisionConstants.POSE_AMBIGUITY_MULTIPLIER);
    // double confidenceMultiplier = Math.max(1, (
    //   Math.max(1,
    //     Math.max(0, smallestDistance - Constants.VisionConstants.NOISY_DISTANCE_METERS)
    //     * Constants.VisionConstants.DISTANCE_WEIGHT
    //   ) * poseAmbiguityFactor)
    // / (1 + ((estimation.targetsUsed.size() - 1) * Constants.VisionConstants.TAG_PRESENCE_WEIGHT)));

    // return Constants.VisionConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
    return smallestDistance <= VisionConstants.NOISY_DISTANCE_METERS;
  }
}