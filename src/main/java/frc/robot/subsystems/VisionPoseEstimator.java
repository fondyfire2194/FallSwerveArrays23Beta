package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Vision.TargetThread1;
import frc.robot.commands.Vision.TargetThread2;

public class VisionPoseEstimator extends SubsystemBase {

  public DriveSubsystem m_drive;

  public AprilTagFieldLayout m_fieldLayout;

  private String fieldFileName = "RapidReact.json";

  private String fieldFileFolder = "/FieldTagLayout/";

  private Pose2d[] visionMeasurement1 = new Pose2d[2];

  private Pose2d[] visionMeasurement2 = new Pose2d[2];

  TargetThread1 tgtTh1;

  private String camera1Name = "cam-IP11";// 10.21.94.11

  public PhotonCamera m_cam1;

  TargetThread2 tgtTh2;

  private String camera2Name = "cam-IP12";// 10.21.94.12

  public PhotonCamera m_cam2;

  int numCams = 2;

  private final int targetsInFle = 24;

  public VisionPoseEstimator(DriveSubsystem drive) {

    m_drive = drive;

    m_cam1 = new PhotonCamera(camera1Name);

    m_cam2 = new PhotonCamera(camera2Name);

    m_cam2.setLED(VisionLEDMode.kOff);

    boolean fieldFileRead = true;

    var f = Filesystem.getDeployDirectory().toString();

    f = f + fieldFileFolder + fieldFileName;

    try {

      m_fieldLayout = new AprilTagFieldLayout(f);

    } catch (IOException e) {

      fieldFileRead = false;

      e.printStackTrace();
    }

    int n = m_fieldLayout.getTags().size();

    fieldFileRead = (n == targetsInFle);

    SmartDashboard.putBoolean("FieldFileRead", fieldFileRead);

    SmartDashboard.putNumber("NumberTagsInFile", n);

    SmartDashboard.putNumber("NumberCameras", numCams);

    if (fieldFileRead) {

      tgtTh1 = new TargetThread1(this, m_cam1, 11);

      if (numCams > 1)
        tgtTh2 = new TargetThread2(this, m_cam2, 21);

    }
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Cam1DriverMode", m_cam1.getDriverMode());

    SmartDashboard.putBoolean("Cam2DriverMode", m_cam2.getDriverMode());

  }

  public void setVisionCorrectionData(int num, int n, Pose2d pose) {
    if (num == 1)
      visionMeasurement1[n] = pose;
    if (num == 2)
      visionMeasurement2[n] = pose;
  }
}
