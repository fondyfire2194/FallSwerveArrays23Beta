package frc.robot.subsystems;

// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionPoseEstimator extends SubsystemBase {

  public Transform3d[] camToTarget = new Transform3d[3];

  public int[] fiducialId = { 0, 0, 0 };

  public int n;

  private Pose3d[] camPose;

  private Pose3d[] targetPose;

  private Pose2d[] visionMeasurement;



  // private PhotonPipelineResult previousPipelineResult;

  // public PhotonCamera m_cam = new PhotonCamera("picam");

  public VisionPoseEstimator() {

    for (int i = 0; i < 2; i++) {
      camPose[i] = new Pose3d();
      targetPose[i] = new Pose3d();
      camToTarget[i] = new Transform3d();
      visionMeasurement[i] = new Pose2d();

    }

  }

  @Override
  public void periodic() {

    // Update pose estimator with visible targets,
    // var pipelineResult = m_cam.getLatestResult();

    // SmartDashboard.putBoolean("HAS TARGETS", pipelineResult.hasTargets());

    // if (!pipelineResult.equals(previousPipelineResult) &&
    // pipelineResult.hasTargets()) {

    // previousPipelineResult = pipelineResult;

    // double imageCaptureTime = Timer.getFPGATimestamp() -
    // (pipelineResult.getLatencyMillis() / 1000d);

    // SmartDashboard.putNumber("ImCapTime", imageCaptureTime);

    // SmartDashboard.putNumber("TargetsSeen", pipelineResult.targets.size());

    // n = 0;

    // for (PhotonTrackedTarget target : pipelineResult.getTargets()) {

    // fiducialId[n] = target.getFiducialId();

    // SmartDashboard.putNumber("FidID " + String.valueOf(n), fiducialId[n]);

    // camToTarget[n] = target.getBestCameraToTarget();

    // camPose[n] = targetPose[n].transformBy(camToTarget[n].inverse());

    // visionMeasurement[n] = camPose[n].transformBy(CAMERA_TO_ROBOT_3D).toPose2d();

    // n++;
    // }
  }

}
