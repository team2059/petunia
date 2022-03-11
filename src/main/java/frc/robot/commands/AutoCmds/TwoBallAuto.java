// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCmds;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuto extends SequentialCommandGroup {
  /** Creates a new TwoBallAuto. */
  public TwoBallAuto(BallCollecterSubsystem ballCollecterSubsystem, DriveTrainSubsystem driveTrainSubsystem,
      Limelight limelight,
      BallCollecterArmSubsystem ballCollecterArmSubsystem, BallCollecterArmSubsystem ballcollecterarmsubsystem2,
      ShooterSubsystem shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
        new MMCollecterArmActivate(ballCollecterArmSubsystem, 3750), new InstantCommand(() -> ballCollecterArmSubsystem
            .getBallCollecterArmTalonSRX()
            .set(ControlMode.PercentOutput, 0)),
        new ParallelCommandGroup(new InitShoot(shooterSubsystem, 17500).withTimeout(3.5),
            new InstantCommand(() -> shooterSubsystem.setIndexSpeed(-0.45)).beforeStarting(new WaitCommand(2))),
        new InstantCommand(() -> ballCollecterSubsystem.setSpeed(-0.5)));

  }

}