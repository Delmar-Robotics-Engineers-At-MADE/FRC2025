// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.ListIterator;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This class represents a sensor for detecting april elements of the game field using PhotonVision. */
public final class PhotonVisionSensor extends SubsystemBase {

  // The name of the network table here MUST match the name specified for the camera in the UI
  static PhotonCamera camera = new PhotonCamera("photoncamera");

  /// @todo Fill this in with the correct measurements later
  //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  static Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));

  static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);    
  static PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

  private Pose2d m_latestEstimatedPose = new Pose2d(0,0,new Rotation2d(0));

  public PhotonVisionSensor() {
    // constructor
    setupDashboard();
  }

  private void setupDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Photon");
    tab.addDouble("Pose X", () -> getPoseX());
    tab.addDouble("Pose Y", () -> getPoseY());
    tab.addString("Rotation", () -> getPoseRot());
  }  

  public void getBestTargetLocation() {
    // Gets all unread results and pulls out an iterator to the last received result
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    PhotonPipelineResult latestResult;
    if (!results.isEmpty()) {
      ListIterator<PhotonPipelineResult> iter = results.listIterator();
      
      while (iter.hasNext()) {
        PhotonPipelineResult temp = iter.next();
        if (temp.hasTargets()) {
          latestResult = temp;
        }
      }
    }

    /// @todo Return the position of the best target in "latestResult"

  }

  /**
   * Get a "noisy" fake global pose reading.
   *
   * @param estimatedRobotPose The robot pose.
   */
  // public static Pose2d getEstimatedGlobalPose(Pose2d estimatedRobotPose) {
  //   var rand =
  //       StateSpaceUtil.makeWhiteNoiseVector(VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
  //   return new Pose2d(
  //       estimatedRobotPose.getX() + rand.get(0, 0),
  //       estimatedRobotPose.getY() + rand.get(1, 0),
  //       estimatedRobotPose.getRotation().plus(new Rotation2d(rand.get(2, 0))));
  // }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    Optional<EstimatedRobotPose> result = Optional.empty();

    if (!results.isEmpty()) {
      ListIterator<PhotonPipelineResult> iter = results.listIterator();
      PhotonPipelineResult latestResult = iter.next();
      
      while (iter.hasNext()) {
        PhotonPipelineResult temp = iter.next();
        //temp.hasTargets()
        if (true) {
          latestResult = temp;
        }
      }

      result = photonPoseEstimator.update(latestResult);
    }

    return result;
  }

  Pose2d getLatestEstimatedPose (Pose2d prevEstimatedRobotPose) {
    Optional<EstimatedRobotPose> poseOption = getEstimatedGlobalPose(prevEstimatedRobotPose);
    if (poseOption.isPresent()) {
      m_latestEstimatedPose = poseOption.get().estimatedPose.toPose2d();
    }
    return m_latestEstimatedPose;
  }

  public double getPoseX(){return m_latestEstimatedPose.getX();}
  public double getPoseY(){return m_latestEstimatedPose.getY();}
  String getPoseRot () {return m_latestEstimatedPose.getRotation().toString();}

}
