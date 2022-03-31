// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.AutoCmds.*;
import frc.robot.commands.PIDShootingCmds.*;
import frc.robot.Constants.*;

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
        public static XboxController logiGameController = new XboxController(1);
        public static Joystick logiFlightController = new Joystick(0);

        private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
        private final ClimberExtenderSubsystem climberExtendSubsystem = new ClimberExtenderSubsystem();
        private final ClimberTiltSubsystem climberTiltSubsystem = new ClimberTiltSubsystem();
        private final static BallCollecterSubsystem ballCollecterSubsystem = new BallCollecterSubsystem();
        private final static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        private final static BallCollecterArmSubsystem ballCollecterArmSubsystem = new BallCollecterArmSubsystem();
        private final Limelight limelight = new Limelight();

        Command autonomousCommand;

        // A chooser for autonomous commands
        // static SendableChooser<Command> pathChooser = new SendableChooser<>();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();
                driveTrainSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveTrainSubsystem));

                // Add commands to the autonomous command chooser

                // boolean isReset = true;

                // String red = "Red";
                // String blue = "Blue";

                // colorChooser.setDefaultOption("Red alliance", red);

                // colorChooser.addOption("Red alliance", red);
                // colorChooser.addOption("Blue alliance", blue);

                // pathChooser.addOption("Complex Auto",
                // loadPathWeaverTrajectoryCommand("pathplanner/generatedJSON/Turn.wpilib.json",
                // isReset));

                // Put the chooser on the dashboard
                // Shuffleboard.getTab("Autonomous").add(pathChooser);
                // Shuffleboard.getTab("Autonomous").add(colorChooser);

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

                // front shooter bumper at tape outer farther goal edge
                // new POVButton(logiGameController, 0).toggleWhenPressed(
                // new Shoot27500(shooterSubsystem, 24000));

                // front shooter bumper at tape inner closest goal edge
                new POVButton(logiGameController, 0).toggleWhenPressed(
                                new FastShoot(shooterSubsystem, 23000));

                // hold left bumper to aim/align with target
                new JoystickButton(logiGameController, Button.kLeftBumper.value)
                                .whileHeld(new AutoAlignCmd(limelight, driveTrainSubsystem));

                // hold right bumper to fire ball by activating indexer
                new JoystickButton(logiGameController, Button.kRightBumper.value)
                                .whileHeld(
                                                new InstantCommand(() -> shooterSubsystem.setIndexSpeed(-0.66)))
                                .whenReleased(new InstantCommand(() -> shooterSubsystem
                                                .setIndexSpeed(0)));

                // spit out - B
                new JoystickButton(logiGameController, Button.kB.value)
                                .whileHeld(() -> ballCollecterSubsystem.setSpeed(0.6))
                                .whenReleased(() -> ballCollecterSubsystem.setSpeed(0));

                // collect - X
                new JoystickButton(logiGameController, Button.kX.value)
                                .whileHeld(() -> ballCollecterSubsystem.setSpeed(-0.6))
                                .whenReleased(() -> ballCollecterSubsystem.setSpeed(0));

                // collecter arm down - A
                new JoystickButton(logiGameController, Button.kA.value)
                                .whenPressed(// new ParallelCommandGroup(
                                             // new InstantCommand(() -> ballCollecterSubsystem.setSpeed(-0.66)),
                                                new MMCollecterArmActivate(ballCollecterArmSubsystem, 1850)
                                                                .andThen(
                                                                                // new InstantCommand(() ->
                                                                                // ballCollecterSubsystem
                                                                                // .setSpeed(0)),
                                                                                new InstantCommand(
                                                                                                () -> ballCollecterArmSubsystem
                                                                                                                .getBallCollecterArmTalonSRX()
                                                                                                                .set(ControlMode.PercentOutput,
                                                                                                                                0))));

                // collecter arm up - Y
                new JoystickButton(logiGameController, Button.kY.value)
                                .whenPressed(new MMCollecterArmActivate(ballCollecterArmSubsystem,
                                                0));

                // driver 2 climbing extreme 3d

                new JoystickButton(logiFlightController, 1).whileHeld(new SlowedArcadeDrive(driveTrainSubsystem));

                new JoystickButton(logiFlightController, 5).whenPressed(

                                new SequentialCommandGroup(
                                                new SMClimberExtendCmd(climberExtendSubsystem, 137).withTimeout(2),
                                                new MMClimberTilt(climberTiltSubsystem, 900)
                                                                .withTimeout(1.66),
                                                new SMClimberExtendCmd(climberExtendSubsystem, 1).withTimeout(1.75),
                                                new MMClimberTilt(climberTiltSubsystem, 466)
                                                                .withTimeout(1.5),
                                                new SMClimberExtendCmd(climberExtendSubsystem, 127.5)
                                                                .withTimeout(2.25),
                                                new MMClimberTilt(climberTiltSubsystem, 0).withTimeout(2.25)));

                new JoystickButton(logiFlightController, 6).whenPressed(
                                new SMClimberExtendCmd(climberExtendSubsystem, 190)
                                                .withTimeout(1.25));

                new JoystickButton(logiFlightController, 4).whenPressed(
                                new MMClimberTilt(climberTiltSubsystem, 0)
                                                .withTimeout(2));

                new JoystickButton(logiFlightController, 3).whenPressed(

                                new SequentialCommandGroup(
                                                new MMClimberTilt(climberTiltSubsystem, 250).withTimeout(0.66),
                                                new ParallelCommandGroup(
                                                                new MMClimberTilt(climberTiltSubsystem, 550)
                                                                                .withTimeout(1.75),

                                                                new SMClimberExtendCmd(climberExtendSubsystem, 127.5)
                                                                                .withTimeout(1)),
                                                new MMClimberTilt(climberTiltSubsystem, 925).withTimeout(1.5),
                                                new SMClimberExtendCmd(climberExtendSubsystem, 1).withTimeout(1.75),
                                                new MMClimberTilt(climberTiltSubsystem, 466)
                                                                .withTimeout(1.75),
                                                new SMClimberExtendCmd(climberExtendSubsystem, 127.5)
                                                                .withTimeout(2)));

                // before climbing, set collecter arm position upright
                // new JoystickButton(logiFlightController, 5)
                // .whenPressed(new MMCollecterArmActivate(ballCollecterArmSubsystem, 1500));

                // extend up
                new POVButton(logiFlightController, 0)
                                .whileHeld(new SMClimberExtendCmd(climberExtendSubsystem, 190))
                                .whenReleased(() -> climberExtendSubsystem
                                                .stopMotors());

                // extend up while tilting forward
                new POVButton(logiFlightController, 45)
                                .whileHeld(new ParallelCommandGroup(new MMClimberTilt(climberTiltSubsystem, 800),
                                                new SMClimberExtendCmd(climberExtendSubsystem, 147.5)))
                                .whenReleased(new ParallelCommandGroup(
                                                new InstantCommand(() -> climberTiltSubsystem.stopMotors()),
                                                new InstantCommand(() -> climberExtendSubsystem
                                                                .stopMotors())));

                // tilt forward
                new POVButton(logiFlightController, 90)
                                .whileHeld(new MMClimberTilt(climberTiltSubsystem,
                                                800))
                                .whenReleased(() -> climberTiltSubsystem.stopMotors());

                // extend down while tilting forward
                new POVButton(logiFlightController, 135)
                                .whileHeld(new ParallelCommandGroup(new MMClimberTilt(climberTiltSubsystem, 800),
                                                new SMClimberExtendCmd(climberExtendSubsystem, 1)))
                                .whenReleased(new ParallelCommandGroup(
                                                new InstantCommand(() -> climberTiltSubsystem.stopMotors()),
                                                new InstantCommand(() -> climberExtendSubsystem
                                                                .stopMotors())));

                // extend down
                new POVButton(logiFlightController, 180)
                                .whileHeld(new SMClimberExtendCmd(climberExtendSubsystem, 1))
                                .whenReleased(() -> climberExtendSubsystem.stopMotors());

                // extend down while tilting back
                new POVButton(logiFlightController, 225)
                                .whileHeld(new ParallelCommandGroup(new MMClimberTilt(climberTiltSubsystem, 0),
                                                new SMClimberExtendCmd(climberExtendSubsystem, 1)))
                                .whenReleased(new ParallelCommandGroup(
                                                new InstantCommand(() -> climberTiltSubsystem.stopMotors()),
                                                new InstantCommand(() -> climberExtendSubsystem
                                                                .stopMotors())));

                // tilt back
                new POVButton(logiFlightController, 270)
                                .whileHeld(new MMClimberTilt(climberTiltSubsystem, 0))
                                .whenReleased(() -> climberTiltSubsystem.stopMotors());

                // extend up while tilting back
                new POVButton(logiFlightController, 315)
                                .whileHeld(new ParallelCommandGroup(new MMClimberTilt(climberTiltSubsystem, 0),
                                                new SMClimberExtendCmd(climberExtendSubsystem, 147.5)))
                                .whenReleased(new ParallelCommandGroup(
                                                new InstantCommand(() -> climberTiltSubsystem.stopMotors()),
                                                new InstantCommand(() -> climberExtendSubsystem
                                                                .stopMotors())));

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

                // return pathChooser.getSelected();
                // TODO 3 ball auto
                return new SequentialCommandGroup(new TwoBallAuto(ballCollecterSubsystem, driveTrainSubsystem,
                                // limelight,
                                ballCollecterArmSubsystem, ballCollecterArmSubsystem,
                                shooterSubsystem),
                                loadPathWeaverTrajectoryCommand(
                                                "pathplanner/generatedJSON/ForwardAuto.wpilib.json",
                                                true),
                                // new AutoAlignCmd(driveTrainSubsystem, limelight),
                                new FinalShoot(shooterSubsystem, 23000).withTimeout(3));

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

        public static BallCollecterArmSubsystem getBallCollecterArmSubsystem() {
                return ballCollecterArmSubsystem;
        }

        public static BallCollecterSubsystem getBallCollecterSubsystem() {
                return ballCollecterSubsystem;
        }

        public static ShooterSubsystem getShooterSubsystem() {
                return shooterSubsystem;
        }

}