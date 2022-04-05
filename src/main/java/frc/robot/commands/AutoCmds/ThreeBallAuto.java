// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCmds;

import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallAuto extends SequentialCommandGroup {

  public Command loadPathWeaverTrajectoryCommand(String filename, boolean resetOdometry) {

    Trajectory trajectory;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + filename, ex.getStackTrace());
      System.out.println("Unable to read from file " + filename);
      return new InstantCommand();
    }

    Command ramseteCommand = new RamseteCommand(
        trajectory,
        RobotContainer.getDriveTrainSubsystem()::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        RobotContainer.getDriveTrainSubsystem()::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        RobotContainer.getDriveTrainSubsystem()::tankDriveVolts,
        RobotContainer.getDriveTrainSubsystem());

    // Run path following command, then stop at the end.
    // If told to reset odometry, reset odometry before running path.
    if (resetOdometry) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> RobotContainer.getDriveTrainSubsystem()
              .resetOdometry(trajectory.getInitialPose())),
          ramseteCommand);
    } else {
      return ramseteCommand;
    }

  }

  /** Creates a new TwoBallAuto. */
  public ThreeBallAuto(BallCollecterSubsystem ballCollecterSubsystem, DriveTrainSubsystem driveTrainSubsystem,
      Limelight limelight,
      BallCollecterArmSubsystem ballCollecterArmSubsystem,
      ShooterSubsystem shooterSubsystem) {

    addCommands(
        new MMCollecterArmActivate(ballCollecterArmSubsystem, 1850),
        new InstantCommand(
            () -> ballCollecterArmSubsystem.getBallCollecterArmTalonSRX().set(ControlMode.PercentOutput, 0)),
        loadPathWeaverTrajectoryCommand("pathplanner/generatedJSON/Second Ball.wpilib.json", true),
        new AutoAlignCmd(limelight, driveTrainSubsystem).withTimeout(1),
        new ParallelCommandGroup(
            new ShootCmd(shooterSubsystem, limelight).withTimeout(5),
            new InstantCommand(() -> shooterSubsystem.setIndexSpeed(-0.75)).beforeStarting(new WaitCommand(2))),
        loadPathWeaverTrajectoryCommand("pathplanner/generatedJSON/ThirdBallPartOne.wpilib.json", false),
        loadPathWeaverTrajectoryCommand(
            "src/main/deploy/pathplanner/generatedJSON/ThirdBallPartTwo.wpilib.json", false),
        new AutoAlignCmd(limelight, driveTrainSubsystem).withTimeout(1),
        new ParallelCommandGroup(
            new ShootCmd(shooterSubsystem, limelight).withTimeout(5),
            new InstantCommand(() -> shooterSubsystem.setIndexSpeed(-0.75)).beforeStarting(new WaitCommand(2)))

    );

  }

}
