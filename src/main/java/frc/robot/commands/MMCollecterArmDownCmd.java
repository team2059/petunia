// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BallCollecterArmSubsystem;

public class MMCollecterArmDownCmd extends CommandBase {
  private final BallCollecterArmSubsystem ballCollecterArm;
  private int targetMin;

  /** Creates a new CollecterArmUp. */
  public MMCollecterArmDownCmd(BallCollecterArmSubsystem ballCollecterArm, int targetMin) {

    this.ballCollecterArm = ballCollecterArm;
    this.targetMin = targetMin;
    addRequirements(ballCollecterArm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Motion Magic */

    /* 4096 ticks/rev * 10 Rotations in either direction */
    // double targetPos = targetMin;
    // * 4096 * 10.0;
    BallCollecterArmSubsystem.getBallCollecterArmTalonSRX().set(ControlMode.MotionMagic, targetMin);

  }

  public int getTargetMin() {
    return targetMin;
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
