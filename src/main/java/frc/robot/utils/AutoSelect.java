// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DoNothing;
import frc.robot.commands.Test.MessageShuffleboard;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class AutoSelect {

    SwerveAutoBuilder autoBuilder;

    List<PathPlannerTrajectory> pathGroup;

    public Command autonomousCommand = new DoNothing();

    public final SendableChooser<Integer> m_autoChooser = new SendableChooser<Integer>();

    // This is just an example event map. It would be better to have a constant,
    // global event map
    // in your code that can be used repeatedly.
    HashMap<String, Command> eventMap = new HashMap<>();

    private boolean skipPathGroup;

    private double startTime;

    public AutoSelect(DriveSubsystem m_drive) {
        eventMap.put("eventone", new MessageShuffleboard("Event A"));
        eventMap.put("eventtwo", new MessageShuffleboard("Event B"));
        eventMap.put("eventthree", new MessageShuffleboard("Event C"));
        eventMap.put("eventfour", new MessageShuffleboard("Event D"));
        eventMap.put("eventfive", new MessageShuffleboard("Event E"));
        eventMap.put("eventsix", new MessageShuffleboard("Event F"));

        // Create the AutoBuilder. This only needs to be created once when robot code
        // starts,
        // not every time you want to create an auto command. A good place to put this
        // is
        // in RobotContainer along with your subsystems.

        autoBuilder = new SwerveAutoBuilder(
                m_drive::getEstimatedPose, // null,
                m_drive::setOdometry, // null,
                DriveConstants.m_kinematics, // null,
                new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X
                                                 // and Y
                                                 // PID controllers)
                new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the
                                                 // rotation
                                                 // controller)
                m_drive::setModuleStates, // Module states consumer used to output to the drive subsystem
                eventMap,
                m_drive);

        m_autoChooser.setDefaultOption("Do Nothing", 0);

        m_autoChooser.addOption("Drive Forward", 1);    

        m_autoChooser.addOption("Drive Straight", 2);

        m_autoChooser.addOption("Five Ball Auto 2", 3);



        SmartDashboard.putData("Auto Selector", m_autoChooser);
    }

    public Command getAutonomousCommand() {

        startTime = Timer.getFPGATimestamp();

        skipPathGroup = false;

        switch (m_autoChooser.getSelected()) {

            case 0:

                skipPathGroup = true;

                autonomousCommand = new DoNothing();

                break;

            case 1:

                pathGroup = PathPlanner.loadPathGroup("DriveForward", new PathConstraints(3, 3));

                break;

                case 2:

                pathGroup = PathPlanner.loadPathGroup("Drive Straight", new PathConstraints(3, 3));

                break;

                case 3:

                pathGroup = PathPlanner.loadPathGroup("FiveBallAuto-2", new PathConstraints(3, 3));

                break;

            default:

                break;

        }
        if (!skipPathGroup)

            autonomousCommand = autoBuilder.fullAuto(pathGroup);

        return autonomousCommand;

    }

}
