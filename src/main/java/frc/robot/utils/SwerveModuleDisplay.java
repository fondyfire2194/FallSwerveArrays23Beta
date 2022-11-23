// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class SwerveModuleDisplay implements Sendable {

    private final DriveSubsystem m_drive;

    SendableBuilder builder;

    public SwerveModuleDisplay(DriveSubsystem drive) {

        m_drive = drive;
    
    }

    @Override
    public void initSendable(SendableBuilder builder) {
       builder.setSmartDashboardType("SwerveModules");

        builder.addBooleanProperty("m_fl_can_ok", m_drive.m_frontLeft::checkCAN, null);
        builder.addBooleanProperty("m_fl_turn_stopped", m_drive.m_frontLeft::turnIsStopped, null);

        builder.addDoubleProperty("m_fl_turn_veocity", m_drive.m_frontLeft::getTurnVelocity, null);
        builder.addDoubleProperty("m_fl_turn_angle", m_drive.m_frontLeft::getTurnPosition, null);

        builder.addDoubleProperty("m_fl_drive_veocity", m_drive.m_frontLeft::getDriveVelocity, null);
        builder.addDoubleProperty("m_fl_drive_position", m_drive.m_frontLeft::getDrivePosition, null);

        builder.addBooleanProperty("m_fr_can_ok", m_drive.m_frontRight::checkCAN, null);
        builder.addBooleanProperty("m_fr_turn_stopped", m_drive.m_frontRight::turnIsStopped, null);

        builder.addDoubleProperty("m_fr_turn_veocity", m_drive.m_frontRight::getTurnVelocity, null);
        builder.addDoubleProperty("m_fr_turn_angle", m_drive.m_frontRight::getTurnPosition, null);

        builder.addDoubleProperty("m_fr_drive_veocity", m_drive.m_frontRight::getDriveVelocity, null);
        builder.addDoubleProperty("m_fr_drive_position", m_drive.m_frontRight::getDrivePosition, null);

    }
}
