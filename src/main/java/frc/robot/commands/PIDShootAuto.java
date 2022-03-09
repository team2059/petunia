// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class PIDShootAuto extends CommandBase {
  private static ShooterSubsystem mShooter;
  private static double rpmVelocity;

  /** Creates a new PIDShootCmd. */
  public PIDShootAuto(ShooterSubsystem mShooter, double rpmVelocity) {
    // Use addRequirements() here to declare submShooter dependencies.
    this.mShooter = mShooter;
    // rpmVelocity in ticks/units 100ms
    this.rpmVelocity = rpmVelocity;
    addRequirements(mShooter);
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.setShooterVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    int counter = 0;

    // if (RobotContainer.logiGameController.getLeftBumperPressed()) {
    // counter++;
    // if (counter % 2 == 0) {
    // return true;
    // }
    // }
    return false;
  }
}
