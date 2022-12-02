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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionPoseEstimator;

/** Add your docs here. */
// This is the private constructor that will be called once by getInstance() and
// it
// should instantiate anything that will be required by the class
public class TargetThread {
    private DriveSubsystem m_drive;
    private VisionPoseEstimator m_vpe;
    private PhotonPipelineResult previousPipelineResult;
    private int n;
    private Pose3d[] camPose = new Pose3d[3];

    private Pose3d[] targetPose = new Pose3d[3];

    private Pose2d[] visionMeasurement = new Pose2d[2];

    public Transform3d[] camToTarget = new Transform3d[3];

    public int[] fiducialId = { 0, 0, 0 };

    public int numberTargets = 0;

    private final int maxTargets = 3;

    int loopctr;

    public TargetThread(DriveSubsystem drive, VisionPoseEstimator vpe) {
        m_drive = drive;
        m_vpe = vpe;
        Thread tagThread = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    while (!Thread.currentThread().isInterrupted()) {

                        execute();

                        Thread.sleep(100);
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });

        // Set up thread properties and start it off

        tagThread.setPriority(Thread.MIN_PRIORITY);
        tagThread.setName("AprilTagThread");
        tagThread.start();

    }

    public void execute() {
        // Update pose estimator with visible targets,
        
        SmartDashboard.putNumber("LPCTE ", loopctr++);
        var pipelineResult = m_vpe.m_cam.getLatestResult();

        SmartDashboard.putBoolean("HAS TARGETS", pipelineResult.hasTargets());

        if (!pipelineResult.equals(previousPipelineResult) &&

                pipelineResult.hasTargets()) {

            previousPipelineResult = pipelineResult;

            numberTargets = pipelineResult.targets.size();

            double imageCaptureTime = pipelineResult.getLatencyMillis() / 1000d;

            SmartDashboard.putNumber("ImCap mSec", imageCaptureTime);

            SmartDashboard.putNumber("#TargetsSeen", numberTargets);

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

}