package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionPoseEstimator extends SubsystemBase {

  private String cameraName = "picam";

  public AprilTagFieldLayout m_fieldLayout;

  private String fieldFileName = "RapidReact.json";

  private String fieldFileFolder = "/FieldTagLayout/";

  public Transform3d CAMERA_TO_ROBOT_3D = new Transform3d();

  public PhotonCamera m_cam = new PhotonCamera(cameraName);

  public VisionPoseEstimator() {

    boolean fieldFileRead = true;

    var f = Filesystem.getDeployDirectory().toString();

    f = f + fieldFileFolder + fieldFileName;

    SmartDashboard.putString("PATHNAME", f);

    try {

      m_fieldLayout = new AprilTagFieldLayout(f);

    } catch (IOException e) {

      fieldFileRead = false;

      e.printStackTrace();
    }

    int n = m_fieldLayout.getTags().size();

    SmartDashboard.putBoolean("FieldFileRead", fieldFileRead);
    SmartDashboard.putNumber("NumberTagsInFile", n);

  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("DriverMode", m_cam.getDriverMode());

  }
}
