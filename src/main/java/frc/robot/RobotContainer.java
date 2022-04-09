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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
import frc.robot.commands.ShootAtTicksCmds.ShootAtTicksCmdFour;
import frc.robot.commands.ShootAtTicksCmds.ShootAtTicksCmdOne;
import frc.robot.commands.ShootAtTicksCmds.ShootAtTicksCmdThree;
import frc.robot.commands.ShootAtTicksCmds.ShootAtTicksCmdTwo;
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

        private final static DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
        private final ClimberExtenderSubsystem climberExtendSubsystem = new ClimberExtenderSubsystem();
        private final ClimberTiltSubsystem climberTiltSubsystem = new ClimberTiltSubsystem();
        private final static BallCollecterSubsystem ballCollecterSubsystem = new BallCollecterSubsystem();
        private final static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        private final static BallCollecterArmSubsystem ballCollecterArmSubsystem = new BallCollecterArmSubsystem();
        private final static Limelight limelight = new Limelight();
        Command autonomousCommand;
        SendableChooser<Command> autoChooser = new SendableChooser<>();

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

                // colorChooser.setDefaultOption("Red alliance", red);

                // colorChooser.addOption("Red alliance", red);
                // colorChooser.addOption("Blue alliance", blue);

                // pathChooser.addOption("Complex Auto",
                // loadPathWeaverTrajectoryCommand("pathplanner/generatedJSON/Turn.wpilib.json",
                // isReset));

                // Put the chooser on the dashboard
                // Shuffleboard.getTab("Autonomou s").add(pathChooser);
                // Shuffleboard.getTab("Autonomous").add(colorChooser);

                autoChooser.setDefaultOption("2 ball", new TwoBallAuto(ballCollecterSubsystem, driveTrainSubsystem,
                                limelight,
                                ballCollecterArmSubsystem,
                                shooterSubsystem));

                autoChooser.addOption("no align 2 ball", new TwoBallNoAlignAuto(ballCollecterSubsystem,
                                driveTrainSubsystem, limelight, ballCollecterArmSubsystem, shooterSubsystem));

                autoChooser.addOption("3 ball", new ThreeBallAuto(ballCollecterSubsystem, driveTrainSubsystem,
                                limelight, ballCollecterArmSubsystem, shooterSubsystem));

                Shuffleboard.getTab("Autonomous").add(autoChooser);

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

                // auto shoot
                new POVButton(logiGameController, 0).toggleWhenPressed(
                new VisionShootCmd(shooterSubsystem, limelight));

                // 7 feet manual shoot
                new POVButton(logiGameController, 90).toggleWhenPressed(
                                new ShootAtTicksCmdTwo(shooterSubsystem, 11325, 10575));

                // 8 feet manual shoot
                new POVButton(logiGameController, 180).toggleWhenPressed(
                                new ShootAtTicksCmdThree(shooterSubsystem, 11425, 11075));

                // 9 feet manual shoot
                new POVButton(logiGameController, 270).toggleWhenPressed(
                                new ShootAtTicksCmdFour(shooterSubsystem, 11575, 11075));

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
                                .whileHeld(() -> ballCollecterSubsystem.setSpeed(0.9))
                                .whenReleased(() -> ballCollecterSubsystem.setSpeed(0));

                // collect - X
                new JoystickButton(logiGameController, Button.kX.value)
                                .whileHeld(() -> ballCollecterSubsystem.setSpeed(-0.9))
                                .whenReleased(() -> ballCollecterSubsystem.setSpeed(0));

                // new JoystickButton(logiGameController, 8)
                // .whileHeld(new InstantCommand(() -> shooterSubsystem.setIndexSpeed(-1)))
                // .whenReleased(() -> shooterSubsystem.setIndexSpeed(0));

                // collecter arm down - A
                new JoystickButton(logiGameController, Button.kA.value)
                                .whenPressed(new SequentialCommandGroup(
                                                new MMCollecterArmActivate(ballCollecterArmSubsystem, 1850),
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
                                                new SMClimberExtendCmd(climberExtendSubsystem, 25).withTimeout(1),
                                                new MMClimberTilt(climberTiltSubsystem, -240).withTimeout(1),
                                                new SMClimberExtendCmd(climberExtendSubsystem, 145).withTimeout(2.5),
                                                new MMClimberTilt(climberTiltSubsystem, 50).withTimeout(1.5),
                                                new SMClimberExtendCmd(climberExtendSubsystem, 1).withTimeout(1.5),
                                                new SMClimberExtendCmd(climberExtendSubsystem, 145).withTimeout(2.5)));

                new JoystickButton(logiFlightController, 6).whenPressed(
                                new SequentialCommandGroup(
                                                new MMClimberTilt(climberTiltSubsystem, -700).withTimeout(2),
                                                new SMClimberExtendCmd(climberExtendSubsystem, 200).withTimeout(2.5),
                                                new MMClimberTilt(climberTiltSubsystem, -400).withTimeout(1.5),
                                                new SMClimberExtendCmd(climberExtendSubsystem, 100).withTimeout(2),
                                                new MMClimberTilt(climberTiltSubsystem, 50).withTimeout(1.5),
                                                new SMClimberExtendCmd(climberExtendSubsystem, 1).withTimeout(2),
                                                new SMClimberExtendCmd(climberExtendSubsystem, 145).withTimeout(2.5)));

                // extend up
                new POVButton(logiFlightController, 0)
                                .whileHeld(new SMClimberExtendCmd(climberExtendSubsystem, 200))
                                .whenReleased(() -> climberExtendSubsystem
                                                .stopMotors());

                // extend up while tilting forward
                new POVButton(logiFlightController, 45)
                                .whileHeld(new ParallelCommandGroup(new MMClimberTilt(climberTiltSubsystem, 50),
                                                new SMClimberExtendCmd(climberExtendSubsystem, 156)))
                                .whenReleased(new ParallelCommandGroup(
                                                new InstantCommand(() -> climberTiltSubsystem.stopMotors()),
                                                new InstantCommand(() -> climberExtendSubsystem
                                                                .stopMotors())));

                // tilt forward
                new POVButton(logiFlightController, 90)
                                .whileHeld(new MMClimberTilt(climberTiltSubsystem,
                                                50))
                                .whenReleased(() -> climberTiltSubsystem.stopMotors());

                // extend down while tilting forward
                new POVButton(logiFlightController, 135)
                                .whileHeld(new ParallelCommandGroup(new MMClimberTilt(climberTiltSubsystem, 50),
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
                                .whileHeld(new ParallelCommandGroup(new MMClimberTilt(climberTiltSubsystem, -700),
                                                new SMClimberExtendCmd(climberExtendSubsystem, 1)))
                                .whenReleased(new ParallelCommandGroup(
                                                new InstantCommand(() -> climberTiltSubsystem.stopMotors()),
                                                new InstantCommand(() -> climberExtendSubsystem
                                                                .stopMotors())));

                // tilt back
                new POVButton(logiFlightController, 270)
                                .whileHeld(new MMClimberTilt(climberTiltSubsystem, -700))
                                .whenReleased(() -> climberTiltSubsystem.stopMotors());

                // extend up while tilting back
                new POVButton(logiFlightController, 315)
                                .whileHeld(new ParallelCommandGroup(new MMClimberTilt(climberTiltSubsystem, -700),
                                                new SMClimberExtendCmd(climberExtendSubsystem, 156)))
                                .whenReleased(new ParallelCommandGroup(
                                                new InstantCommand(() -> climberTiltSubsystem.stopMotors()),
                                                new InstantCommand(() -> climberExtendSubsystem
                                                                .stopMotors())));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         * @throws IOException
         */

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();

        }

        public static DriveTrainSubsystem getDriveTrainSubsystem() {
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

        public static Limelight getLimelight() {
                return limelight;
        }

}