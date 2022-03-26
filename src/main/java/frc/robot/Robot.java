// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  public RobotContainer getRobotContainer() {
    return m_robotContainer;
  }

  // private DriveTrainSubsystem driveTrainSubsystem;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    m_robotContainer.getBallCollecterArmSubsystem().reset();
    m_robotContainer.getClimberExtenderSubsystem().reset();
    m_robotContainer.getClimberTiltSubsystem().reset();
    m_robotContainer.getDriveTrainSubsystem().zeroHeading();
    m_robotContainer.getDriveTrainSubsystem().resetEncoders();

    m_robotContainer.getBallCollecterArmSubsystem().getBallCollecterArmTalonSRX().set(ControlMode.PercentOutput, 0);
    m_robotContainer.getClimberExtenderSubsystem().getRightMotor().set(0);
    m_robotContainer.getClimberExtenderSubsystem().getLeftMotor().set(0);
    m_robotContainer.getClimberTiltSubsystem().getClimberLeftTiltSRX().set(ControlMode.PercentOutput, 0);
    m_robotContainer.getClimberTiltSubsystem().getClimberRightTiltSRX().set(ControlMode.PercentOutput, 0);
    m_robotContainer.getBallCollecterSubsystem().getCollecterIndexer().set(ControlMode.PercentOutput, 0);
    m_robotContainer.getShooterSubsystem().setShooterVelocity(0);
    m_robotContainer.getShooterSubsystem().oppositeFlywheel.set(ControlMode.PercentOutput, 0);
    m_robotContainer.getBallCollecterSubsystem().setVictorSpeed(0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // m_robotContainer.getBallCollecterArmSubsystem().getBallCollecterArmTalonSRX().setSelectedSensorPosition(0);
    // m_robotContainer.getBallCollecterArmSubsystem().getBallCollecterArmTalonSRX().set(ControlMode.PercentOutput,
    // 0);
    // m_robotContainer.getDriveTrainSubsystem().zeroHeading();
    // m_robotContainer.getDriveTrainSubsystem().resetEncoders();
    // m_robotContainer.getClimberExtenderSubsystem().getClimberLeftExtendSRX().set(ControlMode.PercentOutput,
    // 0);
    // m_robotContainer.getClimberExtenderSubsystem().getClimberRightExtendSRX().set(ControlMode.PercentOutput,
    // 0);
    // m_robotContainer.getClimberTiltSubsystem().getClimberLeftTiltSRX().set(ControlMode.PercentOutput,
    // 0);
    // m_robotContainer.getClimberTiltSubsystem().getClimberRightTiltSRX().set(ControlMode.PercentOutput,
    // 0);
    // m_robotContainer.getShooterSubsystem().setShooterVelocity(0);
    // m_robotContainer.getDriveTrainSubsystem().zeroHeading();
    // m_robotContainer.getDriveTrainSubsystem().resetEncoders();

  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    System.out.println("init 0");

    Rotation2d thetaPose = m_robotContainer.getDriveTrainSubsystem().navX.getRotation2d();

    System.out.println("init 1");
    DifferentialDriveOdometry myDriveOdometry = m_robotContainer.getDriveTrainSubsystem().getOdometry();

    System.out.println("init 2");
    myDriveOdometry.resetPosition(new Pose2d(), thetaPose);

    System.out.println("init 3");

    m_robotContainer.getBallCollecterArmSubsystem().reset();

    System.out.println("init 4");
    m_robotContainer.getClimberExtenderSubsystem().reset();

    System.out.println("init 5");
    m_robotContainer.getClimberTiltSubsystem().reset();

    System.out.println("init 6");
    m_robotContainer.getDriveTrainSubsystem().zeroHeading();

    System.out.println("init 7");
    m_robotContainer.getDriveTrainSubsystem().resetEncoders();

    System.out.println("init 8");

    m_robotContainer.getBallCollecterArmSubsystem().getBallCollecterArmTalonSRX().set(ControlMode.PercentOutput, 0);

    System.out.println("init 9");
    m_robotContainer.getClimberExtenderSubsystem().getRightMotor().set(0);

    System.out.println("init 10");
    m_robotContainer.getClimberExtenderSubsystem().getLeftMotor().set(0);

    System.out.println("init 11");
    m_robotContainer.getClimberTiltSubsystem().getClimberLeftTiltSRX().set(ControlMode.PercentOutput, 0);

    System.out.println("init 12");
    m_robotContainer.getClimberTiltSubsystem().getClimberRightTiltSRX().set(ControlMode.PercentOutput, 0);

    System.out.println("init 13");
    m_robotContainer.getShooterSubsystem().setShooterVelocity(0);

    System.out.println("init 14");
    m_robotContainer.getBallCollecterSubsystem().setVictorSpeed(0);

    System.out.println("init 15");

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    System.out.println("init 16");

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {

      m_autonomousCommand.schedule();
      System.out.println("is  not null");
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_robotContainer.getBallCollecterSubsystem().getCollecterIndexer().set(ControlMode.PercentOutput,
    0.33);
    m_robotContainer.getBallCollecterSubsystem().getCollecterMotorTalonSRX().set(ControlMode.PercentOutput,
    -0.66);

    // if (driveTrainSubsystem.getHeading() != 0) {
    // driveTrainSubsystem.zeroHeading();
    // }

  }

  @Override
  public void teleopInit() {

    m_robotContainer.getShooterSubsystem().autoLoader();
    m_robotContainer.getBallCollecterArmSubsystem().getBallCollecterArmTalonSRX().set(ControlMode.PercentOutput, 0);

    m_robotContainer.getClimberExtenderSubsystem().getRightMotor().set(0);
    m_robotContainer.getClimberExtenderSubsystem().getLeftMotor().set(0);

    m_robotContainer.getClimberTiltSubsystem().getClimberLeftTiltSRX().set(ControlMode.PercentOutput, 0);
    m_robotContainer.getClimberTiltSubsystem().getClimberRightTiltSRX().set(ControlMode.PercentOutput, 0);
    m_robotContainer.getBallCollecterSubsystem().getCollecterMotorTalonSRX().set(ControlMode.PercentOutput, 0);
    m_robotContainer.getShooterSubsystem().setShooterVelocity(0);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.getShooterSubsystem().autoLoader();
    m_robotContainer.getBallCollecterSubsystem().setVictorSpeed(0.33);

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
