// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Test.MessageShuffleboard;
import frc.robot.commands.Vision.SetDriverMode;
import frc.robot.commands.swerve.JogDriveModule;
import frc.robot.commands.swerve.JogTurnModule;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionPoseEstimator;
import frc.robot.utils.FFDisplay;
import frc.robot.utils.LEDControllerI2C;

public class RobotContainer {
  // The robot's subsystems
  final DriveSubsystem m_drive = new DriveSubsystem();

  public VisionPoseEstimator m_vpe = new VisionPoseEstimator(m_drive);

  public FFDisplay ff1 = new FFDisplay("test");

  LEDControllerI2C lcI2;

  public final FieldSim m_fieldSim = new FieldSim(m_drive);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  // The driver's controller

  static Joystick leftJoystick = new Joystick(OIConstants.kDriverControllerPort);

  private XboxController m_coDriverController = new XboxController(OIConstants.kCoDriverControllerPort);

  final PowerDistribution m_pdp = new PowerDistribution();

  final GamepadButtons codriver = new GamepadButtons(m_coDriverController, true);
  SwerveAutoBuilder autoBuilder;

  List<PathPlannerTrajectory> pathGroup;
  // temp controller for testing -matt
  // private PS4Controller m_ps4controller = new PS4Controller(1);
  // public PoseTelemetry pt = new PoseTelemetry();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Preferences.removeAll();
    Pref.deleteUnused();

    Pref.addMissing();

    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

    LiveWindow.disableAllTelemetry();
    // Configure the button bindings

    m_fieldSim.initSim();

    // m_ls = new LightStrip(9, 60);

    // lc = LEDController.getInstance();
    lcI2 = LEDControllerI2C.getInstance();

    initializeAutoChooser();

    createSomething();

    // PortForwarder.add(5800, "10.21.94.11", 5800);
    // PortForwarder.add(1181, "10.21.94.11", 1181);
    // PortForwarder.add(1182, "10.21.94.11", 1182);
    // PortForwarder.add(1183, "10.21.94,11", 1183);
    // PortForwarder.add(1184, "10.21.94.11", 1184);

    // () -> -m_coDriverController.getRawAxis(1),
    // () -> -m_coDriverController.getRawAxis(0),
    // () -> -m_coDriverController.getRawAxis(4)));
    // m_drive.setDefaultCommand(
    // new SetSwerveDrive(
    // m_drive,
    // () -> m_ps4controller.getRawAxis(1),
    // () -> m_ps4controller.getRawAxis(0),
    // () -> m_ps4controller.getRawAxis(2)));

    SmartDashboard.putData("SetDriverMode1", new SetDriverMode(m_vpe.m_cam1, true));
    SmartDashboard.putData("ResetDriverMode1", new SetDriverMode(m_vpe.m_cam1, false));

    SmartDashboard.putData("SetDriverMode2", new SetDriverMode(m_vpe.m_cam2, true));
    SmartDashboard.putData("ResetDriverMode2", new SetDriverMode(m_vpe.m_cam2, false));

    m_drive.setDefaultCommand(
        new SetSwerveDrive(
            m_drive,
            () -> leftJoystick.getRawAxis(1),
            () -> leftJoystick.getRawAxis(0),
            () -> leftJoystick.getRawAxis(2)));

    codriver.leftTrigger.whileTrue(new JogTurnModule(
        m_drive,
        () -> -m_coDriverController.getRawAxis(1),
        () -> m_coDriverController.getRawAxis(0),
        () -> m_coDriverController.getRawAxis(2),
        () -> m_coDriverController.getRawAxis(3)));

    // individual modules
    codriver.leftBumper.whileTrue(new JogDriveModule(
        m_drive,
        () -> -m_coDriverController.getRawAxis(1),
        () -> m_coDriverController.getRawAxis(0),
        () -> m_coDriverController.getRawAxis(2),
        () -> m_coDriverController.getRawAxis(3),
        true));

    // all modules
    codriver.rightBumper.whileTrue(new JogDriveModule(
        m_drive,
        () -> -m_coDriverController.getRawAxis(1),
        () -> m_coDriverController.getRawAxis(0),
        () -> m_coDriverController.getRawAxis(2),
        () -> m_coDriverController.getRawAxis(3),
        false));

    JoystickButton button_8 = new JoystickButton(leftJoystick, 8);
    JoystickButton button_7 = new JoystickButton(leftJoystick, 7);

    // position turn modules individually
    // driver.X_button.whenPressed(new PositionTurnModule(m_drive,
    // ModulePosition.FRONT_LEFT));
    // driver.A_button.whenPressed(new PositionTurnModule(m_drive,
    // ModulePosition.FRONT_RIGHT));
    // driver.B_button.whenPressed(new PositionTurnModule(m_drive,
    // ModulePosition.BACK_LEFT));
    // driver.Y_button.whenPressed(new PositionTurnModule(m_drive,
    // ModulePosition.BACK_RIGHT));

  }

  private void initializeAutoChooser() {
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));

    SmartDashboard.putData("Auto Selector", m_autoChooser);

  }

  public void simulationPeriodic() {

    m_fieldSim.periodic();
  }

  public void periodic() {
    m_fieldSim.periodic();
  }

  public double getThrottle() {
    return -leftJoystick.getThrottle();
  }

  public void createSomething() {

    pathGroup = PathPlanner.loadPathGroup("Drive Straight", new PathConstraints(1, 1));

    // This is just an example event map. It would be better to have a constant,
    // global event map
    // in your code that can be used repeatedly.
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("shooterStart", new MessageShuffleboard("ShooterStart", 5000));
    eventMap.put("intakeDown", new MessageShuffleboard("Intake Down", 1));
    eventMap.put("intakeOn", new MessageShuffleboard("Intake Run", 2));
    eventMap.put("intakeOff", new MessageShuffleboard("Intake Stop", 3));
    eventMap.put("turnToTarget", new MessageShuffleboard("DriveTurnTotarget", 0));
    eventMap.put("shoot", new MessageShuffleboard("ShooterShoot", 10));

    // Create the AutoBuilder. This only needs to be created once when robot code
    // starts,
    // not every time you want to create an auto command. A good place to put this
    // is
    // in RobotContainer along with your subsystems.

    autoBuilder = new SwerveAutoBuilder(
        m_drive::getEstimatedPose, // null,
        m_drive::setOdometry, // null,
        DriveConstants.m_kinematics, // null,
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y
                                         // PID controllers)
        new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation
                                         // controller)
        m_drive::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        m_drive);

  }

  public Command getAutonomousCommand() {
    return autoBuilder.fullAuto(pathGroup);
  }
}