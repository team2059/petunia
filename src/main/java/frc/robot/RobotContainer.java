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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
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
import frc.robot.subsystems.ClimberExtenderSubsystem;
import frc.robot.subsystems.ClimberTiltSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.JoystickArcadeDriveCmd;

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
        public static Joystick joystick = new Joystick(0);

        private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
        private final ClimberExtenderSubsystem climberExtendSubsystem = new ClimberExtenderSubsystem();
        private final ClimberTiltSubsystem climberTiltSubsystem = new ClimberTiltSubsystem();
        private final static BallCollecterSubsystem ballCollecterSubsystem = new BallCollecterSubsystem();
        private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        private final static BallCollecterArmSubsystem ballCollecterArmSubsystem = new BallCollecterArmSubsystem();
        private final Limelight limelight = new Limelight();

        Command autonomousCommand;

        // A chooser for autonomous commands
        SendableChooser<Command> pathChooser = new SendableChooser<>();

        // A chooser for autonomous commands
        static SendableChooser<String> colorChooser = new SendableChooser<>();

        public static String getAllianceColor() {
                return colorChooser.getSelected().toString();
        }

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();
                //ARCADE DRIVE CHANGED TO USE JOYSTICK FOR LIMELIGHT TEST CHANGE BACK !!!!!!!
                driveTrainSubsystem.setDefaultCommand(new JoystickArcadeDriveCmd(driveTrainSubsystem));

                // Add commands to the autonomous command chooser

                boolean isReset = true;

                String red = "Red";
                String blue = "Blue";

                colorChooser.setDefaultOption("Red alliance", red);

                colorChooser.addOption("Red alliance", red);
                colorChooser.addOption("Blue alliance", blue);

                pathChooser.addOption("Path forward Auto",
                                loadPathWeaverTrajectoryCommand(
                                                "pathplanner/generatedJSON/Forward.wpilib.json",
                                                isReset));

                // pathChooser.addOption("Complex Auto",
                // loadPathWeaverTrajectoryCommand("pathplanner/generatedJSON/Turn.wpilib.json",
                // isReset));

                // Put the chooser on the dashboard
                Shuffleboard.getTab("Autonomous").add(pathChooser);
                Shuffleboard.getTab("Autonomous").add(colorChooser);

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

                // should be -1 being nothing pressed, 0 being top, 1 being top
                // right, 2 being right, and progressing clockwise to 7 in top left.

                // back button spins shooter up
                new JoystickButton(xboxController, Button.kBack.value).whenPressed(
                                new PIDShootCmd(shooterSubsystem, 500));

                // start button degrees does different rpm
                new JoystickButton(xboxController, Button.kStart.value).whenPressed(
                                new PIDShootCmd(shooterSubsystem, 3570));

                // right bumper actuates indexer
                new JoystickButton(xboxController, Button.kRightBumper.value)
                                .whenPressed(new InstantCommand(() -> shooterSubsystem.setIndexSpeed(-0.66)));

                // , new WaitCommand(2.5),
                // new InstantCommand(() -> shooterSubsystem.setIndexSpeed(-0.66))))
                // new ConditionalCommand(new InstantCommand(() ->
                // shooterSubsystem.setIndexSpeed(-0.66)),
                // new InstantCommand(() -> shooterSubsystem.setIndexSpeed(0)),
                // PIDShootCmd.isAtTargetVelocity())));

                // spit out - B
                new JoystickButton(xboxController, Button.kB.value)
                                .whileHeld(() -> ballCollecterSubsystem.setSpeed(0.66))
                                .whenReleased(() -> ballCollecterSubsystem.setSpeed(0));

                // collect - X
                new JoystickButton(xboxController, Button.kX.value)
                                .whileHeld(() -> ballCollecterSubsystem.setSpeed(-0.66))
                                .whenReleased(() -> ballCollecterSubsystem.setSpeed(0));

                // collecter arm up - Y
                new JoystickButton(xboxController, Button.kY.value)
                                .whenPressed(new MMCollecterArmActivate(ballCollecterArmSubsystem, 3500));

                // collecter arm down - A
                new JoystickButton(xboxController, Button.kA.value)
                                .whenPressed(new MMCollecterArmActivate(ballCollecterArmSubsystem,
                                                0));

                // driver 2 climbing (only need 5 bindings + switch arcade drive to driver 2's
                // controller when actuated)

                // extend up
                new JoystickButton(logitech, Button.kY.value)
                                .whileHeld(new MMClimberExtend(climberExtendSubsystem, 77500))
                                .whenReleased(() -> climberExtendSubsystem.stopMotors());

                // extend down
                new JoystickButton(logitech, Button.kA.value)
                                .whileHeld(new MMClimberExtend(climberExtendSubsystem, 0))
                                .whenReleased(() -> climberExtendSubsystem.stopMotors());

                // TODO Tilt mechanics
                // tilt forward
                new JoystickButton(logitech, Button.kX.value)
                                .whileHeld(new MMClimberTilt(climberTiltSubsystem,
                                                325))
                                .whenReleased(() -> climberTiltSubsystem.stopMotors());
                // tilt back
                new JoystickButton(logitech, Button.kB.value)
                                .whileHeld(new MMClimberTilt(climberTiltSubsystem, 0))
                                .whenReleased(() -> climberTiltSubsystem.stopMotors());

                // tilt align

                new JoystickButton(logitech, Button.kLeftBumper.value)
                                .whenPressed(new MMClimberTilt(climberTiltSubsystem, 175))
                                .whenReleased(() -> climberTiltSubsystem.stopMotors());

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

                // // shoot
                // new JoystickButton(logitech, Axis.kRightTrigger.value)
                // .whenPressed(new ConditionalCommand(new ShootBallCmd(shooterSubsystem),
                // new SequentialCommandGroup(
                // new MMCollecterArmActivate(ballCollecterArmSubsystem,
                // 0),
                // new InstantCommand(() -> ShooterSubsystem
                // .setIndexSpeed(-0.66))),
                // ballCollecterArmSubsystem.isSameColor()));

                //limelight auto align/ranging buttons
                new JoystickButton(joystick, 1).whileHeld(new AutoAlignCmd(driveTrainSubsystem, limelight));

                new JoystickButton(joystick, 3).whileHeld(new AutoRangeCmd(driveTrainSubsystem, limelight, 49.0));
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
                // return new SequentialCommandGroup(
                // new ParallelCommandGroup(new
                // MMCollecterArmActivate(ballCollecterArmSubsystem, 0),
                // new InstantCommand(() -> ballCollecterArmSubsystem
                // .setCollecterArmSpeed(-.5)),
                // pathChooser.getSelected()),
                // new ParallelCommandGroup(new InstantCommand(() ->
                // shooterSubsystem.setIndexSpeed(-.5)),
                // new InstantCommand(() -> shooterSubsystem.setShooterVelocity(0.75))));

                return pathChooser.getSelected();

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

}
