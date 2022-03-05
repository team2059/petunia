// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberExtendConstants;
import frc.robot.subsystems.BallCollecterArmSubsystem;
import frc.robot.subsystems.ClimberExtenderSubsystem;

public class MMClimberExtend extends CommandBase {
  private final ClimberExtenderSubsystem climberExtend;
  private int target;

  /** Creates a new CollecterArmUp. */
  public MMClimberExtend(ClimberExtenderSubsystem climberExtend, int target) {

    this.climberExtend = climberExtend;
    this.target = target;
    addRequirements(climberExtend);
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

<<<<<<< HEAD
    ClimberExtenderSubsystem.getClimberLeftExtendSRX().set(ControlMode.MotionMagic, target,
        DemandType.ArbitraryFeedForward, 0.1);
    ClimberExtenderSubsystem.getClimberRightExtendSRX().set(ControlMode.MotionMagic, target,
        DemandType.ArbitraryFeedForward, 0.1);
=======
    ClimberExtenderSubsystem.getClimberLeftExtendSRX().set(ControlMode.MotionMagic, target);
    ClimberExtenderSubsystem.getClimberRightExtendSRX().set(ControlMode.MotionMagic, target);
>>>>>>> 74c695c37a5ed790eb3afccab8e0938960a4e1f8

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // set output to 0 to use coast mode
    // we want to
    // chagne control mode to Percent output to use coast mode
    // ClimberExtenderSubsystem.getClimberLeftExtendSRX().set(ControlMode.PercentOutput,
    // 0);
    // ClimberExtenderSubsystem.getClimberLeftExtendSRX().setNeutralMode(NeutralMode.Coast);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // double position = (
    // ClimberExtenderSubsystem.getClimberLeftExtendSRX().getSelectedSensorPosition());

    // if position (usually -40 to -60) is less than 0, end command, we want to
    // chagne control mode to Percent output to use coast mode
    // return (position < target);
    return false;

  }
}
