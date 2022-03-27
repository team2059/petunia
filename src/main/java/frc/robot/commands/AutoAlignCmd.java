// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoAlignCmd extends CommandBase {

  private final DriveTrainSubsystem driveTrainSubsystem;
  private final Limelight limelight;

  double lastXOffset;

  /** Creates a new AutoAlign. */
  public AutoAlignCmd(DriveTrainSubsystem driveTrainSubsystem, Limelight limelight) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.limelight = limelight;

    addRequirements(driveTrainSubsystem, limelight);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double tx = limelight.getXOffset();
    boolean tv = limelight.getHasTargets();
    double speed = 0.0;

    if (tv) {
      // TODO - Check Area processing v X-Ofset / Y-Offset
      speed = ((limelight.getXOffset() - 2.5) * 0.075); // P gain (minus one to align to 1 degree instead of zero)

      if (speed > 1.5) { // set speed limits
        speed = 1.5;
      } else if (speed < -1.5) {
        speed = -1.5;
      }
      lastXOffset = limelight.getXOffset();
    } else {
      if (lastXOffset < 0) {
        speed = -0.5;
      } else {
        speed = 0.5;
      }
      // TODO Forward Distance testing
    }

    // System.out.println("SPEED: " + speed);
    driveTrainSubsystem.arcadeDrive(0, -(speed * 0.5));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (limelight.getXOffset() < 2.0 && limelight.getXOffset() > -2.0);
  }
}
