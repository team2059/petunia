// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallCollecter;

public class SpitOutBall extends CommandBase {

  private final BallCollecter ballCollecter;
  private double speed;

  /** Creates a new IntakeBallCmd. */
  public SpitOutBall(BallCollecter subsystem, double speed) {
    this.ballCollecter = subsystem;
    this.speed = speed;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // TODO spit out ball -> just set a negative speed
    ballCollecter.setCollectorMotorSpeed(-speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballCollecter.setCollectorMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
