// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PIDShootingCmds;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCmd extends CommandBase {
  private static ShooterSubsystem shooterSubsystem;
  private static Limelight limelight;

  /** Creates a new PIDShootCmd. */
  public ShootCmd(ShooterSubsystem shooterSubsystem, Limelight limelight) {
    // Use addRequirements() here to declare subshooterSubsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    this.limelight = limelight;

    addRequirements(shooterSubsystem);
  }

  public double setPrimary() {
    double distance = limelight.getTargetDistance();
    double primaryTicks = 9485 + (240 * distance);
    return primaryTicks;
  }
  public double setSecondary() {
    double distance = limelight.getTargetDistance();
    double secondaryTicks = 7778.4 + (387.6 * distance);
    return secondaryTicks;
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
    shooterSubsystem.ballShooter.set(ControlMode.Velocity, setPrimary());
    shooterSubsystem.oppositeFlywheel.set(ControlMode.Velocity, setSecondary());
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
