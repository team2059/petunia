// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestSequenceCmd extends SequentialCommandGroup {
  /** Creates a new TestSequenceCmd. */
  public TestSequenceCmd(BallCollecterSubsystem ballCollecterSubsystem, DriveTrainSubsystem driveTrainSubsystem,
      Limelight limelight,
      BallCollecterArmSubsystem ballCollecterArmSubsystem,
      ShooterSubsystem shooterSubsystem, ClimberExtenderSubsystem climberExtenderSubsystem,
      ClimberTiltSubsystem climberTiltSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(new InstantCommand(() -> driveTrainSubsystem.tankDriveVolts(-1, -1)).withTimeout(2),
        new InstantCommand(() -> driveTrainSubsystem.tankDriveVolts(1, 1)).withTimeout(2),
        new InstantCommand(() -> driveTrainSubsystem.tankDriveVolts(1, -1)).withTimeout(2),
        new InstantCommand(() -> driveTrainSubsystem.tankDriveVolts(-1, 1)).withTimeout(2),
        new MMCollecterArmActivate(ballCollecterArmSubsystem, 1850),
        new InstantCommand(
            () -> ballCollecterArmSubsystem.getBallCollecterArmTalonSRX().set(ControlMode.PercentOutput, 0)),
        new InstantCommand(() -> ballCollecterSubsystem.setSpeed(-0.66)).withTimeout(2),
        new ShootAtTicksCmd(shooterSubsystem, 28000, 28000).withTimeout(5),
        new MMClimberTilt(climberTiltSubsystem, 900).withTimeout(5),
        new SMClimberExtendCmd(climberExtenderSubsystem, 190));
  }
}
