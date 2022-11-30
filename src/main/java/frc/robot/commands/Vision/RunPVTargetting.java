// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionPoseEstimator;

public class RunPVTargetting extends CommandBase {
  /** Creates a new RunPVTargetting. */
  private VisionPoseEstimator m_vpe;
  private DriveSubsystem m_drive;
  private PhotonPipelineResult previousPipelineResult;
  private int n;
  private Pose3d[] camPose = new Pose3d[3];

  private Pose3d[] targetPose = new Pose3d[3];

  private Pose2d[] visionMeasurement = new Pose2d[2];

  public Transform3d[] camToTarget = new Transform3d[3];

  public int[] fiducialId = { 0, 0, 0 };

  public RunPVTargetting(VisionPoseEstimator vpe, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_vpe = vpe;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (int i = 0; i < 2; i++) {
      camPose[i] = new Pose3d();
      targetPose[i] = new Pose3d();
      camToTarget[i] = new Transform3d();
      visionMeasurement[i] = new Pose2d();

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update pose estimator with visible targets,
    var pipelineResult = m_vpe.m_cam.getLatestResult();

    SmartDashboard.putBoolean("HAS TARGETS", pipelineResult.hasTargets());

    if (!pipelineResult.equals(previousPipelineResult) &&

        pipelineResult.hasTargets()) {

      previousPipelineResult = pipelineResult;

      double imageCaptureTime = Timer.getFPGATimestamp() -
          (pipelineResult.getLatencyMillis() / 1000d);

      SmartDashboard.putNumber("ImCapTime", imageCaptureTime);

      SmartDashboard.putNumber("#TargetsSeen", pipelineResult.targets.size());

      n = 0;

      for (PhotonTrackedTarget target : pipelineResult.getTargets()) {

        fiducialId[n] = target.getFiducialId();

        SmartDashboard.putNumber("FidID " + String.valueOf(n), fiducialId[n]);

        Optional<Pose3d> temp = m_vpe.m_fieldLayout.getTagPose(fiducialId[n]);

        if (temp.isPresent())

          targetPose[n] = temp.get();

        else

          break;

        camToTarget[n] = target.getBestCameraToTarget();

        camPose[n] = targetPose[n].transformBy(camToTarget[n].inverse());

        visionMeasurement[n] = camPose[n].transformBy(m_vpe.CAMERA_TO_ROBOT_3D).toPose2d();

        m_drive.m_poseEstimator.addVisionMeasurement(visionMeasurement[n], imageCaptureTime);

        n++;
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
