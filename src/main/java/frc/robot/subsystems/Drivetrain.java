// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private Module m_frontLeftModule;
  private Module m_frontRightModule;
  private Module m_backLeftModule;
  private Module m_backRightModule;

  private Rotation2d fieldAngle = new Rotation2d();

  private final Field2d field2d = new Field2d();

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    Constants.Drivetrain.KINEMATICS,
    getAngle()
  );

  /** Creates a new Drivetrain. */
  public Drivetrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
      getAngle(),
      m_frontLeftModule.getState(),
      m_frontRightModule.getState(),
      m_backLeftModule.getState(),
      m_backRightModule.getState()
    );

    field2d.setRobotPose(odometry.getPoseMeters());
  }

  public void drive(double x, double y, double rot, boolean fieldRelative) {
    SwerveModuleState[] states = Constants.Drivetrain.KINEMATICS.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getAngle()) : 
        new ChassisSpeeds(x, y, rot)// check if there is a requirement for offsetting
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.MAX_SPEED);
    setModuleStates(states[0], states[1], states[2], states[3]);
  }

  public void setModuleStates(SwerveModuleState fl, SwerveModuleState fr, SwerveModuleState bl, SwerveModuleState br) {
    SwerveDriveKinematics.desaturateWheelSpeeds(new SwerveModuleState[] {fl, fr, bl, br}, Constants.Drivetrain.MAX_SPEED);
    m_frontLeftModule.setDesiredState(fl);
    m_frontRightModule.setDesiredState(fr);
    m_backLeftModule.setDesiredState(bl);
    m_backRightModule.setDesiredState(br);
  }

  public double getGyroAngle() {
    return Math.IEEEremainder(navx.getAngle(), 360.0);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(getGyroAngle());
  }
}
