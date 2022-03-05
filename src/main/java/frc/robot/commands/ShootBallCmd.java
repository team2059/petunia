// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootBallCmd extends CommandBase {
  private final ShooterSubsystem mShooter;

  /** Creates a new ShootBallCmd. */
  public ShootBallCmd(ShooterSubsystem mShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mShooter = mShooter;
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
<<<<<<< HEAD
    mShooter.setIndexSpeed(-0.66);
=======

>>>>>>> 74c695c37a5ed790eb3afccab8e0938960a4e1f8
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< HEAD
    mShooter.setShooterVelocity(0.66);
=======

    mShooter.setShooterVelocity(-0.66);

>>>>>>> 74c695c37a5ed790eb3afccab8e0938960a4e1f8
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (mShooter.getShooterVelocity() == -0.66);
  }
}
