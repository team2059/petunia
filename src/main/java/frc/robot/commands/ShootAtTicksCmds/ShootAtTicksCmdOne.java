// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootAtTicksCmds;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAtTicksCmdOne extends CommandBase {
  private static ShooterSubsystem shooterSubsystem;
  private static double primaryTicks;
  private static double secondaryTicks;

  /** Creates a new PIDShootCmd. */
  public ShootAtTicksCmdOne(ShooterSubsystem shooterSubsystem, double primaryTicks, double secondaryTicks) {
    // Use addRequirements() here to declare submShooter dependencies.
    this.shooterSubsystem = shooterSubsystem;
    // rpmVelocity in ticks/units 100ms
    this.primaryTicks = primaryTicks;
    this.secondaryTicks = secondaryTicks;
    addRequirements(shooterSubsystem);
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
    shooterSubsystem.oppositeFlywheel.set(ControlMode.PercentOutput, secondaryTicks);
    shooterSubsystem.ballShooter.set(ControlMode.Velocity, primaryTicks);
    // mShooter.oppositeFlywheel.set(ControlMode.PercentOutput, -0.75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.oppositeFlywheel.set(ControlMode.PercentOutput, 0);
    shooterSubsystem.setShooterVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // if (RobotContainer.logiGameController.getLeftBumperPressed()) {
    // counter++;
    // if (counter % 2 == 0) {
    // return true;
    // }
    // }
    return false;
  }
}
