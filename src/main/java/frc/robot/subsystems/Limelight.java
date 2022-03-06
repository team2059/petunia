// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math;

public class Limelight extends SubsystemBase {

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  private final NetworkTableEntry xOffset = table.getEntry("tx");
  private final NetworkTableEntry yOffset = table.getEntry("ty");
  private final NetworkTableEntry hasTargets = table.getEntry("tv");

  double goalHeightInches = 104.0;

  double limelightHeightInches = 30.75;

  double limelightMountingAngleDegrees = 27.0;

  public double getDistance() {

    double targetOffsetAngle_Vertical = getYOffset();
    double angleToGoalDegrees = limelightMountingAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightHeightInches) / Math.tan(angleToGoalRadians)
        ;

    return distanceFromLimelightToGoalInches;
  }

  /** Creates a new Limelight. */
  public Limelight() {
  }

  @Override
  public void periodic() {
    double distance = getDistance();
    SmartDashboard.putNumber("Distance Calc", distance);
    // This method will be called once per scheduler run
  }

  public double getXOffset() {
    return xOffset.getDouble(0.0);
  }

  public double getYOffset() {
    return yOffset.getDouble(0.0);
  }

  public boolean getHasTargets() {
    double tv = hasTargets.getDouble(0.0);
    if (tv < 1.0) {
      return false;
    } else {
      return true;
    }

  }

}
