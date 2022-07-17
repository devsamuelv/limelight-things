// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.ThePinkAlliance.core.limelight.Limelight;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

public class LimelightTest extends CommandBase {
  double height = Units.inchesToMeters(9.3 / 2.54);
  double targetHeight = Units.inchesToMeters(40);
  double cameraPitch = Units.degreesToRadians(45);
  double targetPitch = Units.degreesToRadians(90);

  Limelight m_limelight;
  PhotonCamera camera = new PhotonCamera(NetworkTableInstance.getDefault(), "test");

  /** Creates a new LimelightTest. */
  public LimelightTest(Limelight m_limelight) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_limelight = m_limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera.setLED(VisionLEDMode.kOn);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    List<PhotonTrackedTarget> targets = this.camera.getLatestResult().targets;
    PhotonTrackedTarget bestTarget = this.camera.getLatestResult().getBestTarget();

    if (targets.size() > 0) {
      // I Think this works when 3d mode is enabled.
      // Transform2d distance = targets.get(0).getCameraToTarget();

      double range = PhotonUtils.calculateDistanceToTargetMeters(height, targetHeight, cameraPitch,
          Units.degreesToRadians(bestTarget.getPitch()));

      SmartDashboard.putNumber("Distance Meters", range);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    camera.setLED(VisionLEDMode.kOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
