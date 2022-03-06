// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class PIDShootCmd extends CommandBase {
  private static ShooterSubsystem mShooter;
  private static double rpmVelocity;

  /** Creates a new PIDShootCmd. */
  public PIDShootCmd(ShooterSubsystem mShooter, double rpmVelocity) {
    // Use addRequirements() here to declare submShooter dependencies.
    this.mShooter = mShooter;
    this.rpmVelocity = rpmVelocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


    // 500 rpm
    // max unit/100ms = ~
    // rpmVelocity = rpmVelocity * 4096 / 600;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooter.ballShooter.set(ControlMode.Velocity, rpmVelocity);
  }

  public static BooleanSupplier isAtTargetVelocity() {
    return () -> ((rpmVelocity - mShooter.getShooterVelocity()) > 0
        && (rpmVelocity - mShooter.getShooterVelocity()) < 25);
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
