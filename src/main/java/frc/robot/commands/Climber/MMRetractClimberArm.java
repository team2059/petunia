// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberAngleSubsystem;

public class MMRetractClimberArm extends CommandBase {
  private final ClimberAngleSubsystem climberAngle;
  private int targetMin;
  /** Creates a new MMRetractClimberArm. */
  public MMRetractClimberArm(ClimberAngleSubsystem climberAngle, int targetMin) {
    this.climberAngle = climberAngle;
    this.targetMin = targetMin;
    addRequirements(climberAngle);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ClimberAngleSubsystem.getClimberAngleTalonSRX().set(ControlMode.MotionMagic, targetMin);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
