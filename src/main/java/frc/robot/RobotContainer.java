// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.MMClimberExtendDownCmd;
import frc.robot.commands.MMClimberExtendUpCmd;
import frc.robot.commands.MMClimberTiltBackCmd;
import frc.robot.commands.MMClimberTiltForwardCmd;
import frc.robot.commands.MMCollecterArmDownCmd;
import frc.robot.commands.MMCollecterArmUpCmd;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.BallCollecterArmSubsystem;
import frc.robot.subsystems.BallCollecterSubsystem;
import frc.robot.subsystems.ClimberExtenderSubsystem;
import frc.robot.subsystems.ClimberTiltSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
        public static XboxController logitech = new XboxController(3);
        public static XboxController xboxController = new XboxController(5);

        private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
        private final ClimberExtenderSubsystem climberExtendSubsystem = new ClimberExtenderSubsystem();
        private final ClimberTiltSubsystem climberTiltSubsystem = new ClimberTiltSubsystem();
        private final BallCollecterSubsystem ballCollecterSubsystem = new BallCollecterSubsystem();
        private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        private final BallCollecterArmSubsystem ballCollecterArmSubsystem = new BallCollecterArmSubsystem();

        Command autonomousCommand;

        // A chooser for autonomous commands
        SendableChooser<Command> m_chooser = new SendableChooser<>();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();
                driveTrainSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveTrainSubsystem));

                // Add commands to the autonomous command chooser

                boolean isReset = true;

                m_chooser.addOption("Path forward Auto",
                                loadPathWeaverTrajectoryCommand("pathplanner/generatedJSON/Forward.wpilib.json",
                                                isReset));

                m_chooser.addOption("Complex Auto",
                                loadPathWeaverTrajectoryCommand("pathplanner/generatedJSON/Turn.wpilib.json", isReset));

                // Put the chooser on the dashboard
                Shuffleboard.getTab("Autonomous").add(m_chooser);

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

                // collect - X
                new JoystickButton(xboxController, Button.kX.value)
                                .whileHeld(() -> ballCollecterSubsystem.setSpeed(0.66))
                                .whenReleased(() -> ballCollecterSubsystem.setSpeed(0));

                // spit out - B
                new JoystickButton(xboxController, Button.kB.value)
                                .whileHeld(() -> ballCollecterSubsystem.setSpeed(-0.66))
                                .whenReleased(() -> ballCollecterSubsystem.setSpeed(0));

                // collecter arm up - Y
                new JoystickButton(xboxController, Button.kY.value)
                                .whenPressed(new MMCollecterArmUpCmd(ballCollecterArmSubsystem, 2300));

                // collecter arm down - A
                new JoystickButton(xboxController, Button.kA.value)
                                .whenPressed(new MMCollecterArmDownCmd(ballCollecterArmSubsystem,
                                                0));

                // extend up
                new JoystickButton(logitech, Button.kY.value)
                                .whileHeld(new MMClimberExtendUpCmd(climberExtendSubsystem, 77900))
                                .whenReleased(() -> climberExtendSubsystem.stopMotors());

                // extend down
                new JoystickButton(logitech, Button.kA.value)
                                .whileHeld(new MMClimberExtendDownCmd(climberExtendSubsystem, 50))
                                .whenReleased(() -> climberExtendSubsystem.stopMotors());

                // TODO Tilt mechanics
                // tilt forward
                new JoystickButton(logitech, Button.kX.value)
                                .whileHeld(new MMClimberTiltForwardCmd(climberTiltSubsystem,
                                                600));
                // .whenReleased(() -> ClimberTiltSubsystem.getClimberLeftTiltSRX()
                // .set(ControlMode.PercentOutput, 0.000001));

                // tilt back
                new JoystickButton(logitech, Button.kB.value)
                                .whileHeld(new MMClimberTiltBackCmd(climberTiltSubsystem, 0))

                ;

                // tilt align

                new JoystickButton(logitech, Button.kLeftBumper.value)
                                .whenPressed(new MMClimberTiltForwardCmd(climberTiltSubsystem, -114))

                ;

                // .whenReleased(() -> ClimberTiltSubsystem.getClimberLeftTiltSRX()
                // .set(ControlMode.PercentOutput, 0.000001));

                // X - indexer forward
                // new JoystickButton(logitech, Button.kX.value).whileHeld(() ->
                // shooterSubsystem.setIndexSpeed(.5))
                // .whenReleased(() -> shooterSubsystem.setIndexSpeed(0));

                // B - indexer backwards
                // new JoystickButton(logitech, Button.kB.value).whileHeld(() ->
                // shooterSubsystem.setIndexSpeed(-.5))
                // .whenReleased(() -> shooterSubsystem.setIndexSpeed(0));

                // new JoystickButton(logitech, Button.kRightBumper.value)
                // .whenPressed(() -> shooterSubsystem.setShooterVelocity(0.75));

                // new JoystickButton(logitech, Button.kRightBumper.value)
                // .whenPressed(() -> shooterSubsystem.setShooterVelocity(100));

        }

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
                                driveTrainSubsystem::getPose,
                                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                                new SimpleMotorFeedforward(
                                                DriveConstants.ksVolts,
                                                DriveConstants.kvVoltSecondsPerMeter,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                DriveConstants.kDriveKinematics,
                                driveTrainSubsystem::getWheelSpeeds,
                                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                                // RamseteCommand passes volts to the callback
                                driveTrainSubsystem::tankDriveVolts,
                                driveTrainSubsystem);

                // Run path following command, then stop at the end.
                // If told to reset odometry, reset odometry before running path.
                if (resetOdometry) {
                        return new SequentialCommandGroup(
                                        new InstantCommand(() -> driveTrainSubsystem
                                                        .resetOdometry(trajectory.getInitialPose())),
                                        ramseteCommand);
                } else {
                        return ramseteCommand;
                }

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         * @throws IOException
         */

        public Command getAutonomousCommand() {

                // TODO test auto routine
                return new SequentialCommandGroup(
                                new ParallelCommandGroup(new MMCollecterArmDownCmd(ballCollecterArmSubsystem, 0),
                                                new InstantCommand(() -> ballCollecterArmSubsystem
                                                                .setCollecterArmSpeed(-.5)),
                                                m_chooser.getSelected()),
                                new ParallelCommandGroup(new InstantCommand(() -> shooterSubsystem.setIndexSpeed(-.5)),
                                                new InstantCommand(() -> shooterSubsystem.setShooterVelocity(0.75))));

                // return m_chooser.getSelected();

        }

        public DriveTrainSubsystem getDriveTrainSubsystem() {
                return driveTrainSubsystem;
        }

        public ClimberTiltSubsystem getClimberTiltSubsystem() {
                return climberTiltSubsystem;
        }

        public ClimberExtenderSubsystem getClimberExtenderSubsystem() {
                return climberExtendSubsystem;
        }

}
