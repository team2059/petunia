// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BallCollecterArmSubsystem;

public class MMCollecterArmActivate extends CommandBase {
  private final BallCollecterArmSubsystem ballCollecterArm;
  private int target;

  /** Creates a new CollecterArmUp. */
  public MMCollecterArmActivate(BallCollecterArmSubsystem ballCollecterArm, int target) {

    this.ballCollecterArm = ballCollecterArm;
    this.target = target;
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
    // double targetPos = target;
    // * 4096 * 10.0;

    BallCollecterArmSubsystem.getBallCollecterArmTalonSRX().set(ControlMode.MotionMagic, target);

  }

  public int getTarget() {
    return target;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // set output to 0 to use coast mode
    // we want to
    // chagne control mode to Percent output to use coast mode
    // BallCollecterArmSubsystem.getBallCollecterArmTalonSRX().set(ControlMode.PercentOutput,
    // 0);
    BallCollecterArmSubsystem.getBallCollecterArmTalonSRX().setNeutralMode(NeutralMode.Coast);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    double position = (BallCollecterArmSubsystem.getBallCollecterArmTalonSRX().getSelectedSensorPosition());

    // if position (usually -40 to -60) is less than 0, end command, we want to
    // chagne control mode to Percent output to use coast mode
    return (position < target);

  }
}
