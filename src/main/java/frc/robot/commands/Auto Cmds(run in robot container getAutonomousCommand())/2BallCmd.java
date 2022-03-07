// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.AutoAlignCmd;
import frc.robot.commands.AutoRangeCmd;
import frc.robot.commands.MMClimberExtend;
import frc.robot.commands.MMClimberTilt;
import frc.robot.commands.MMCollecterArmActivate;
import frc.robot.commands.PIDShootCmd;
import frc.robot.commands.ShootBallCmd;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.BallCollecterArmSubsystem;
import frc.robot.subsystems.BallCollecterSubsystem;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class 2BallCmd extends SequentialCommandGroup {

  

  

  /** Creates a new AutoAlign. */
  public 2BallCmd(DriveTrainSubsystem driveTrainSubsystem, Limelight limelight, BallCollecterArmSubsystem ballCollecterArmSubsystem, BallCollecterSubsystem ballCollecterArm,ShooterSubsystem shooterSubsystem) {
    addCommands();

    
  }

}