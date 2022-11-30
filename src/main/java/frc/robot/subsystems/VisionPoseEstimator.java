package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionPoseEstimator extends SubsystemBase {


  private String filePath = "/home/lvuser/deploy/";

  private String fieldLayoutFileName = "RapdReact";

  private String cameraName = "camera";

  public AprilTagFieldLayout m_fieldLayout;

  public Transform3d CAMERA_TO_ROBOT_3D = new Transform3d();



  public PhotonCamera m_cam = new PhotonCamera(cameraName);

  public VisionPoseEstimator() {


    boolean success = true;

    try {

      m_fieldLayout = new AprilTagFieldLayout(filePath + fieldLayoutFileName + ".json");

    } catch (IOException e) {

      SmartDashboard.putNumber("ENE", 911);

      success = false;

      e.printStackTrace();
    }

    SmartDashboard.putBoolean("SUCCESS", success);

  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("DriverMode", m_cam.getDriverMode());

    

  }
}
