// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivebaseSubsystem;

public class ArcadedriveCmd extends CommandBase {

  private final DrivebaseSubsystem drivetrainSubsystem;

  /** Creates a new teleopDrive. */
  public ArcadedriveCmd(DrivebaseSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting TeleopDriveCommand");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = RobotContainer.joyStick.getY();
    double zRotation = RobotContainer.joyStick.getZ() * 0.5;
    // negate zRotation because we invertetd rightMotorControllerGroup
    drivetrainSubsystem.arcadeDrive(xSpeed, zRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
