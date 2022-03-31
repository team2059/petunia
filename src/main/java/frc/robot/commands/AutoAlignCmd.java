// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.Limelight;

public class AutoAlignCmd extends CommandBase {
  private final Limelight limelight;
  private final DriveTrainSubsystem driveTrainSubsystem;
  private double speed = 0.0;
  private double xAlignP = 0.075;

  /** Creates a new AutoAlign. */
  public AutoAlignCmd(Limelight limelight, DriveTrainSubsystem driveTrainSubsystem) {
    this.limelight = limelight;
    this.driveTrainSubsystem = driveTrainSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    speed = limelight.getTargetAngle() * xAlignP;

    driveTrainSubsystem.arcadeDrive(0, -(speed*0.5));
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
