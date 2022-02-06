// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArcadedriveCmd;
import frc.robot.subsystems.BallCollecter;
import frc.robot.subsystems.BallCollecterArm;
import frc.robot.subsystems.DrivebaseSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    public static Joystick joyStick = new Joystick(0);
    private final DrivebaseSubsystem drivebaseSubsystem = new DrivebaseSubsystem();
    private final ArcadedriveCmd arcadedriveCmd = new ArcadedriveCmd(drivebaseSubsystem);

    private static BallCollecterArm ballCollecterArm = new BallCollecterArm();
    private static BallCollecter ballCollecter = new BallCollecter();

    // private final TeleopDriveCommand teleopDriveCommand = new
    // TeleopDriveCommand(drivebaseSubsystem);

    Command autonomousCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        drivebaseSubsystem.setDefaultCommand(new ArcadedriveCmd(drivebaseSubsystem));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(joyStick, 2).whenPressed(() -> ballCollecterArm.setCollecterArmSpeed(0.1));

        new JoystickButton(joyStick, 3).whenPressed(() -> ballCollecter.setCollectorMotorSpeed(0.1));

        new JoystickButton(joyStick, 3).whenPressed(() -> ballCollecter.setCollectorMotorSpeed(-0.1));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     * @throws IOException
     */

    public Command getAutonomousCommand() throws IOException {

        // set left to inverted so motors go in same direction
        // drivebaseSubsystem.getLeftMotorControllerGroup().setInverted(true);
        // drivebaseSubsystem.resetEncoders();
        // drivebaseSubsystem.zeroHeading();

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        DriveConstants.ksVolts,
                        DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                10);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 0), new Translation2d(1.5, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(2, 0, new Rotation2d(0)),
                // Pass config
                config);

        Path trajectoryPath = Filesystem.getDeployDirectory().toPath()
                .resolve("pathplanner/generatedJSON/forward2.wpilib.json");
        Trajectory trajectory = new Trajectory();
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

        // drivebaseSubsystem.resetOdometry(trajectory.getInitialPose());
        // drivebaseSubsystem.getField().getObject("Pose").setTrajectory(trajectory);
        // Rotation2d rotation2d =
        // drivebaseSubsystem.getPose().minus(trajectory.getInitialPose());

        // Transform2d rotate90 = new Pose2d(0, 0, Rotation2d.fromDegrees(90));
        // trajectory = trajectory.transformBy(new Transform2d(new Translation2d(0.0,
        // 0.0), Rotation2d.fromDegrees(110)));

        // drivebaseSubsystem.resetOdometry(trajectory.getInitialPose());

        // drivebaseSubsystem.differentialDrive.resetOdometry(trajectory.getInitialPose());
        // transforming the PathWeaver trajectory to match your current pose
        // var transform =
        // drivebaseSubsystem.getPose().minus(trajectory.getInitialPose());
        // straightTrajectory = trajectory.transformBy(transform);

        RamseteCommand ramseteCommand = new RamseteCommand(
                trajectory,
                drivebaseSubsystem::getPose,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                        DriveConstants.ksVolts,
                        DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                drivebaseSubsystem::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                drivebaseSubsystem::tankDriveVolts,
                drivebaseSubsystem);

        // Reset odometry to the starting pose of the trajectory.
        // drivebaseSubsystem.resetOdometry(trajectory.getInitialPose());
        drivebaseSubsystem.resetOdometry(trajectory.getInitialPose());
        // Run path following command, then stop at the end via break mode to ensure no
        // voltage and then set motors to idle mode for teleOp.
        return ramseteCommand.andThen(() -> drivebaseSubsystem.setBreakMode())
                .andThen(() -> drivebaseSubsystem.setIdleMode());

    }

}
