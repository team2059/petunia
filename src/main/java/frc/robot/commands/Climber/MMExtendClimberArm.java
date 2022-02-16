// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.subsystems.ClimberExtenderSubsystem;

public class MMExtendClimberArm extends CommandBase {
  private final ClimberExtenderSubsystem climberExtender;
  private int targetMax;

  /** Creates a new MMExtendClimberArm. */
  public MMExtendClimberArm(ClimberExtenderSubsystem climberExtender, int targetMax) {
    this.climberExtender = climberExtender;
    this.targetMax = targetMax;
    addRequirements(climberExtender);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ClimberExtenderSubsystem.getClimberExtenderTalonSRX().set(ControlMode.MotionMagic, targetMax);

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
