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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.AutoAlignCmd;
import frc.robot.commands.AutoRangeCmd;
import frc.robot.commands.MMClimberExtend;
import frc.robot.commands.MMClimberTilt;
import frc.robot.commands.MMCollecterArmActivate;
import frc.robot.commands.SlowArcadeDrive;
import frc.robot.commands.AutoCmds.FinalShoot;
import frc.robot.commands.AutoCmds.TwoBallAuto;
import frc.robot.commands.PIDShootingCmds.Shoot12500;
import frc.robot.commands.PIDShootingCmds.Shoot17500;
import frc.robot.commands.PIDShootingCmds.Shoot22500;
import frc.robot.commands.PIDShootingCmds.Shoot27500;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.BallCollecterArmSubsystem;
import frc.robot.subsystems.BallCollecterSubsystem;
import frc.robot.subsystems.ClimberExtenderSubsystem;
import frc.robot.subsystems.ClimberTiltSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

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
        public static XboxController logiGameController = new XboxController(0);
        public static Joystick logiFlightController = new Joystick(1);

        private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
        private final ClimberExtenderSubsystem climberExtendSubsystem = new ClimberExtenderSubsystem();
        private final ClimberTiltSubsystem climberTiltSubsystem = new ClimberTiltSubsystem();
        private final static BallCollecterSubsystem ballCollecterSubsystem = new BallCollecterSubsystem();
        private final static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        private final static BallCollecterArmSubsystem ballCollecterArmSubsystem = new BallCollecterArmSubsystem();
        private final Limelight limelight = new Limelight();

        Command autonomousCommand;

        // A chooser for autonomous commands
        static SendableChooser<Command> pathChooser = new SendableChooser<>();

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
                driveTrainSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveTrainSubsystem));

                // Add commands to the autonomous command chooser

                boolean isReset = true;

                // String red = "Red";
                // String blue = "Blue";

                // colorChooser.setDefaultOption("Red alliance", red);

                // colorChooser.addOption("Red alliance", red);
                // colorChooser.addOption("Blue alliance", blue);

                pathChooser.addOption("Path forward Auto",
                                loadPathWeaverTrajectoryCommand(
                                                "pathplanner/generatedJSON/Forward.wpilib.json",
                                                isReset));

                // pathChooser.addOption("Complex Auto",
                // loadPathWeaverTrajectoryCommand("pathplanner/generatedJSON/Turn.wpilib.json",
                // isReset));

                // Put the chooser on the dashboard
                Shuffleboard.getTab("Autonomous").add(pathChooser);
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

                new POVButton(logiGameController, 0).toggleWhenPressed(
                                new Shoot27500(shooterSubsystem, 27500));

                new POVButton(logiGameController, 90).toggleWhenPressed(
                                new Shoot22500(shooterSubsystem, 22500));

                new POVButton(logiGameController, 180).toggleWhenPressed(
                                new Shoot17500(shooterSubsystem, 17500));

                new POVButton(logiGameController, 270).toggleWhenPressed(
                                new Shoot12500(shooterSubsystem, 12500));

                // back button spins shooter up
                // new JoystickButton(logiGameController, Button.kLeftBumper.value)

                // right bumper actuates indexer
                new JoystickButton(logiGameController, Button.kRightBumper.value)
                                .whileHeld(new SequentialCommandGroup(new AutoAlignCmd(driveTrainSubsystem, limelight),
                                                new InstantCommand(() -> shooterSubsystem.setIndexSpeed(-0.66))))
                                .whenReleased(new InstantCommand(() -> shooterSubsystem
                                                .setIndexSpeed(0)));

                // spit out - B
                new JoystickButton(logiGameController, Button.kB.value)
                                .whileHeld(() -> ballCollecterSubsystem.setSpeed(0.66))
                                .whenReleased(() -> ballCollecterSubsystem.setSpeed(0));

                // collect - X
                new JoystickButton(logiGameController, Button.kX.value)
                                .whileHeld(() -> ballCollecterSubsystem.setSpeed(-0.66))
                                .whenReleased(() -> ballCollecterSubsystem.setSpeed(0));

                // collecter arm up - Y
                new JoystickButton(logiGameController, Button.kY.value)
                                .whenPressed(new MMCollecterArmActivate(ballCollecterArmSubsystem, 3750)
                                                .andThen(new InstantCommand(() -> ballCollecterArmSubsystem
                                                                .getBallCollecterArmTalonSRX()
                                                                .set(ControlMode.PercentOutput, 0))));

                // collecter arm down - A
                new JoystickButton(logiGameController, Button.kA.value)
                                .whenPressed(new MMCollecterArmActivate(ballCollecterArmSubsystem,
                                                0));

                // driver 2 climbing extreme 3d

                // extend up
                new POVButton(logiFlightController, 0)
                                .whileHeld(new MMClimberExtend(climberExtendSubsystem, 77500))
                                .whenReleased(() -> climberExtendSubsystem
                                                .stopMotors());

                new JoystickButton(logiFlightController, 5)
                                .whenPressed(new MMCollecterArmActivate(ballCollecterArmSubsystem, 1500));

                // extend down
                new POVButton(logiFlightController, 180)
                                .whileHeld(new MMClimberExtend(climberExtendSubsystem, 0))
                                .whenReleased(() -> climberExtendSubsystem.stopMotors());

                // tilt forward
                new POVButton(logiFlightController, 90)
                                .whileHeld(new MMClimberTilt(climberTiltSubsystem,
                                                700))
                                .whenReleased(() -> climberTiltSubsystem.stopMotors());
                // tilt back
                new POVButton(logiFlightController, 270)
                                .whileHeld(new MMClimberTilt(climberTiltSubsystem, 0))
                                .whenReleased(() -> climberTiltSubsystem.stopMotors());

                // tilt align
                new JoystickButton(logiFlightController, 2).whenPressed(new MMClimberTilt(climberTiltSubsystem, 850));

                new JoystickButton(logiFlightController, 1).whileHeld(new SlowArcadeDrive(driveTrainSubsystem));

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
                                limelight,
                                ballCollecterArmSubsystem, ballCollecterArmSubsystem,
                                shooterSubsystem),
                                pathChooser.getSelected(),
                                // new AutoAlignCmd(driveTrainSubsystem, limelight),
                                new FinalShoot(shooterSubsystem, 27500).withTimeout(2.5));

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
