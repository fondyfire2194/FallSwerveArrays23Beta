// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.VisionPoseEstimator;

/** Add your docs here. */
// This is the private constructor that will be called once by getInstance() and
// it
// should instantiate anything that will be required by the class
public class TargetThread2 {

    private VisionPoseEstimator m_vpe;
    private PhotonCamera m_cam;

    private int m_num;
    private int n;
    private Pose3d[] camPose = new Pose3d[3];

    private Pose3d[] targetPose = new Pose3d[3];

    public Transform3d[] camToTarget = new Transform3d[3];

    public int[] fiducialId = { 0, 0, 0 };

    public int numberTargets = 0;

    private final int maxTargets = 2;// ##0,1

    double imageCaptureTime;

    int loopctr;

    List<PhotonTrackedTarget> tpr;

    public TargetThread2(VisionPoseEstimator vpe, PhotonCamera cam, int num) {
        m_vpe = vpe;
        m_cam = cam;

        m_num = num;

        Thread tagThread2 = new Thread(new Runnable() {
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

        tagThread2.setPriority(Thread.NORM_PRIORITY);
        tagThread2.setName("AprilTagThread 2");
        tagThread2.start();

    }

    public void execute() {
        // Update pose estimator with visible targets,

        SmartDashboard.putNumber("LPCTT2 ", loopctr++);// thread running indicator

        var pipelineResult = m_cam.getLatestResult();

        SmartDashboard.putBoolean("HAS TARGETS 2", pipelineResult.hasTargets());

        if (!pipelineResult.hasTargets())

        {

            numberTargets = 0;

            imageCaptureTime = 0;

        } else {

            numberTargets = pipelineResult.targets.size();

            imageCaptureTime = pipelineResult.getLatencyMillis() / 1000d;

            tpr = pipelineResult.targets;

            if (numberTargets > 2)

                tpr = pipelineResult.getTargets().subList(0, maxTargets);

            n = 0;

            for (PhotonTrackedTarget target : tpr) {

                fiducialId[n] = target.getFiducialId();

                SmartDashboard.putNumber("FidID 2 " + String.valueOf(n), fiducialId[n]);

                Optional<Pose3d> temp = m_vpe.m_fieldLayout.getTagPose(fiducialId[n]);

                if (temp.isPresent())

                    targetPose[n] = temp.get();

                else

                    break;

                camToTarget[n] = target.getBestCameraToTarget();

                camPose[n] = targetPose[n].transformBy(camToTarget[n].inverse());

                m_vpe.setVisionCorrectionData(1, n,
                        camPose[n].transformBy(VisionConstants.CAMERA_TO_ROBOT_3D).toPose2d());

                n++;
            }

        }

        SmartDashboard.putNumber("#TargetsSeen 2",
                numberTargets);
        SmartDashboard.putNumber("ImCap mSec_2",
                imageCaptureTime);

    }
}
