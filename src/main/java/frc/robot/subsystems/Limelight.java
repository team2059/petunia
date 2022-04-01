// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Limelight extends SubsystemBase {
  
  PhotonCamera camera = new PhotonCamera("hh");

  private boolean hasTargets = false;
  private double targetDistance = 0.0;
  private double targetAngle = 0.0;

  private double CAMERA_HEIGHT_METERS = Units.feetToMeters(13.75);
  private double TARGET_HEIGHT_METERS = Units.feetToMeters(75);
  private double CAMERA_PITCH_RADIANS = Units.degreesToRadians(28.5);

  final double xalignP = 0.075;

  /** Creates a new Vision. */
  public Limelight() {}

  public boolean hasTarget() {
    return hasTargets;
  }

  public double getTargetDistance() {
    return targetDistance;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  @Override
  public void periodic() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      targetAngle = result.getBestTarget().getYaw();
      targetDistance = Units.metersToFeet(PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, 
        Units.degreesToRadians(result.getBestTarget().getPitch())));
      hasTargets = true;
    }
    // This method will be called once per scheduler run
  }
}
