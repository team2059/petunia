// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.Limelight;
import java.lang.Math;


public class AutoRangeCmd extends CommandBase {

  private final DriveTrainSubsystem driveTrainSubsystem;
  private final Limelight limelight;
  private final double target;

  final double goalHeightInches = 82.0;

  final double limelightHeightInches = 30.75;

  final double limelightMountingAngleDegrees = 26.5;
  




  /** Creates a new DistanceCalcAutoRange. */
  public AutoRangeCmd(DriveTrainSubsystem driveTrainSubsystem, Limelight limelight, double target) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.limelight = limelight;
    this.target = target;
    

    addRequirements(driveTrainSubsystem, limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentDistance = limelight.getDistance();
    double distanceError = currentDistance - target;
    double speed = distanceError * 0.0725;

    driveTrainSubsystem.arcadeDrive(-(speed * 0.5), 0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
