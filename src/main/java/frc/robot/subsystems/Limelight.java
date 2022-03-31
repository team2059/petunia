// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
      targetDistance = 0.0;
      hasTargets = true;
    }
    // This method will be called once per scheduler run
  }
}
