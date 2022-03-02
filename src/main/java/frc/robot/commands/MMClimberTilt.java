// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberTiltSubsystem;

public class MMClimberTilt extends CommandBase {
  private final ClimberTiltSubsystem climberTilt;
  private final int target;

  /** Creates a new MMClimberTiltForwardCmd. */
  public MMClimberTilt(ClimberTiltSubsystem climberTilt, int target) {
    this.climberTilt = climberTilt;
    this.target = target;

    addRequirements(climberTilt);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ClimberTiltSubsystem.getClimberLeftTiltSRX().set(ControlMode.MotionMagic, target);
    ClimberTiltSubsystem.getClimberRightTiltSRX().set(ControlMode.MotionMagic, target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // ClimberTiltSubsystem.getClimberLeftTiltSRX().setNeutralMode(NeutralMode.Brake);
    // ClimberTiltSubsystem.getClimberRightTiltSRX().setNeutralMode(NeutralMode.Brake);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return
    // (ClimberTiltSubsystem.getClimberLeftTiltSRX().getSelectedSensorPosition() <
    // target);
    
    return false;
  }
}
