package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.VisionConstants;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  private PhotonCamera camera;
  private boolean hasTarget;
  private PhotonPipelineResult result;

  private AprilTagFieldLayout aprilTagFieldLayout;
  // if there's a pose estimator in the drivetrain subsystem, update it with this estimator
  private PhotonPoseEstimator poseEstimator;

  public Vision() {
    camera = new PhotonCamera("picam");

    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }

    poseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            VisionConstants.robotToCam);
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = camera.getLatestResult();
    hasTarget = result.hasTargets();
    if (hasTarget) {
      this.result = result;
    }
    Optional<EstimatedRobotPose> currentEstPose = getEstimatedGlobalPose();
    if (currentEstPose.isPresent()) {
      SmartDashboard.putNumber(
          "vision/estimated x pos", currentEstPose.get().estimatedPose.getX());
      SmartDashboard.putNumber(
          "vision/estimated y pos", currentEstPose.get().estimatedPose.getY());
      SmartDashboard.putNumber(
          "vision/estimated z pos", currentEstPose.get().estimatedPose.getZ());
    }
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return poseEstimator.update();
  }

  public PhotonTrackedTarget getBestTarget() {
    if (hasTarget) {
      return result.getBestTarget();
    } else {
      return null;
    }
  }

  public boolean getHasTarget() {
    return hasTarget;
  }

  public PhotonCamera getCam() {
    return camera;
  }
}
