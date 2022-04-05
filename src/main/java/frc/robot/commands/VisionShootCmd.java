// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class VisionShootCmd extends CommandBase {
  private static ShooterSubsystem shooterSubsystem;
  private static Limelight limelight;
  double distance;

  /** Creates a new PIDShootCmd. */
  public VisionShootCmd(ShooterSubsystem shooterSubsystem, Limelight limelight) {
    // Use addRequirements() here to declare subshooterSubsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    this.limelight = limelight;

    addRequirements(shooterSubsystem);
  }

  public double setPrimary(double distance) {
    System.out.println("shoot cmd activated distance: " + distance);
    double primaryTicks = 0;
    if (distance > 7 * 12 && distance < 8 * 12) {
      primaryTicks = 10550 + (8.333 * distance);
    }
    if (distance > 8 * 12 && distance < 9 * 12) {
      primaryTicks = 10150 + (12.5 * distance);
    }
    if (distance > 9 * 12 && distance < 10 * 12) {
      primaryTicks = 7000 + (41.666 * distance);
    }

    return primaryTicks;
  }

  public double setSecondary(double distance) {
    System.out.println("shoot cmd activated distance: " + distance);
    double secondaryTicks = 0;
    if (distance > 7 * 12 && distance < 8 * 12) {
      secondaryTicks = 7000 + (41.666 * distance);
    }
    if (distance > 8 * 12 && distance < 9 * 12) {
      secondaryTicks = 11000;
    }
    if (distance > 9 * 12 && distance < 10 * 12) {
      secondaryTicks = 3872 + (66 * distance);
    }

    return secondaryTicks;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distance = limelight.getTargetDistance();

    // 500 rpm
    // max unit/100ms = ~
    // rpmVelocity = rpmVelocity * 4096 / 600;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.ballShooter.set(ControlMode.Velocity, setPrimary(distance));
    shooterSubsystem.oppositeFlywheel.set(ControlMode.Velocity, setSecondary(distance));
    // shooterSubsystem.oppositeFlywheel.set(ControlMode.PercentOutput, -0.33);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterVelocity(0);
    shooterSubsystem.oppositeFlywheel.set(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}