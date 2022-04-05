// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;

public class TurnToAngleCmd extends CommandBase {
  private final DriveTrainSubsystem driveTrainSubsystem;
  private final double angle;

  /** Creates a new TurnCmd. */
  public TurnToAngleCmd(DriveTrainSubsystem driveTrainSubsystem, double angle) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.angle = angle;
    addRequirements(driveTrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Find the heading error; setpoint is 90
    double error = angle - driveTrainSubsystem.getGyro().getAngle();
    driveTrainSubsystem.tankDriveVolts(0.04 * error, -0.04 * error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
