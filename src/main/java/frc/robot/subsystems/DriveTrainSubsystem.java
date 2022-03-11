// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class DriveTrainSubsystem extends SubsystemBase {

  static CANSparkMax leftFrontCANSparkMax = new CANSparkMax(DriveConstants.leftFrontCANSparkMaxCANId,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  static CANSparkMax leftBackCANSparkMax = new CANSparkMax(DriveConstants.leftbackCANSparkMaxCANId,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  static CANSparkMax rightFrontCANSparkMax = new CANSparkMax(DriveConstants.rightFrontCANSparkMaxCANId,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  static CANSparkMax rightBackCANSparkMax = new CANSparkMax(DriveConstants.rightBackCANSparkMaxCANId,
      CANSparkMaxLowLevel.MotorType.kBrushless);

  WPI_TalonSRX testTalon = new WPI_TalonSRX(0);

  // using built in encoders in NEO motors
  private final static RelativeEncoder leftRelativeEncoder = leftFrontCANSparkMax.getEncoder();

  // NEGATIVE RIGHT ENCODER VALUE!!!!!!!!!!!
  public final static RelativeEncoder rightRelativeEncoder = rightFrontCANSparkMax.getEncoder();

  private final static MotorControllerGroup leftMotorControllerGroup = new MotorControllerGroup(leftFrontCANSparkMax,
      leftBackCANSparkMax);
  public final static MotorControllerGroup rightMotorControllerGroup = new MotorControllerGroup(rightFrontCANSparkMax,
      rightBackCANSparkMax);

  private final static DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorControllerGroup,
      rightMotorControllerGroup);

  private Field2d m_field = new Field2d();

  public Field2d getField() {
    return m_field;
  }

  /** Creates a new DriveTrain. */
  public DriveTrainSubsystem() {

    // SmartDashboard.putData("Field", m_field);

    leftRelativeEncoder.setPosition(0);
    rightRelativeEncoder.setPosition(0);

    leftBackCANSparkMax.follow(leftFrontCANSparkMax);
    rightBackCANSparkMax.follow(rightFrontCANSparkMax);

    // Inverted
    rightMotorControllerGroup.setInverted(false);
    leftMotorControllerGroup.setInverted(true);
    // leftMotorControllerGroup.setInverted(true);
    // leftMotorControllerGroup.setInverted(true);

    leftBackCANSparkMax.restoreFactoryDefaults();
    leftFrontCANSparkMax.restoreFactoryDefaults();
    rightFrontCANSparkMax.restoreFactoryDefaults();
    rightBackCANSparkMax.restoreFactoryDefaults();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // leftMotorControllerGroup.setInverted(true);

    // kLinearDistancePerMotorRotation = gear
    // ratio*2*pi*Units.inchesToMeters(wheel raidus)
    // velocity is / 60 to go from meters/minute to meters/second
    // kLinearDistancePerMotorRotation and velocity is / 60 to go from meters/minute
    // to meters/second

    rightRelativeEncoder.setPositionConversionFactor(DriveConstants.kLinearDistancePerMotorRotation);
    leftRelativeEncoder.setPositionConversionFactor(DriveConstants.kLinearDistancePerMotorRotation);
    rightRelativeEncoder.setVelocityConversionFactor(DriveConstants.kLinearDistancePerMotorRotation / 60);
    leftRelativeEncoder.setVelocityConversionFactor(DriveConstants.kLinearDistancePerMotorRotation / 60);

    navX.reset();
    navX.calibrate();
    resetEncoders();

    // navX.getRotation2d().minus(navX.getRotation2d()).minus(navX.getRotation2d());

    m_odometry = new DifferentialDriveOdometry(navX.getRotation2d());
    m_odometry.resetPosition(new Pose2d(), navX.getRotation2d());
  }

  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");

  @Override
  public void periodic() {

    m_field.setRobotPose(m_odometry.getPoseMeters());

    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderPosition(),
        getRightEncoderPosition());

    var translation = m_odometry.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());

    // SmartDashboard.putNumber("x pose", m_odometry.getPoseMeters().getX());
    // SmartDashboard.putNumber("y pose", m_odometry.getPoseMeters().getY());

    // SmartDashboard.putNumber("theta pose",
    // m_odometry.getPoseMeters().getRotation().getDegrees());

    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Left encoder value in meters",
    // getLeftEncoderPosition());
    // SmartDashboard.putNumber("Right encoder value in meters",
    // getRightEncoderPosition());

    // NEGATE RIGHT ENCODER VALUE!!!!!!!!!!!
    // SmartDashboard.putNumber("Right encoder value in meters",
    // getRightEncoderPosition());
    // SmartDashboard.putNumber("Gyro heading", getHeading());

    // NEGATE RIGHT ENCODER VALUE!!!!!!!!!!!
    m_odometry.update(navX.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());

  }

  public static void driveToDistanceMeters(double meters) {
    if (leftRelativeEncoder.getPosition() < meters) {
      differentialDrive.tankDrive(-.5, -.5);
    } else {
      differentialDrive.tankDrive(0, 0);
    }
  }

  public DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  public MotorControllerGroup getLeftMotorControllerGroup() {
    return leftMotorControllerGroup;
  }

  public double getRightEncoderPosition() {
    return -rightRelativeEncoder.getPosition();
  }

  public double getLeftEncoderPosition() {
    return leftRelativeEncoder.getPosition();
  }

  public double getRightEncoderVelocity() {
    return -rightRelativeEncoder.getVelocity();
  }

  public double getLeftEncoderVelocity() {
    return leftRelativeEncoder.getVelocity();
  }

  public DifferentialDrive getDifferentialDrive() {
    return differentialDrive;
  }

  // The gyro sensor
  public final static Gyro navX = new AHRS(SPI.Port.kMXP);
  // private final static Gyro navX = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  public Gyro getGyro() {
    return navX;
  }
  // private final Gyro navX = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  public DifferentialDriveOdometry getOdomotery() {
    return m_odometry;
  }

  // 2pr/(minutes/rotation)
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    // zeroHeading();
    m_odometry.resetPosition(pose, navX.getRotation2d());
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // using explciity declared encoders
    // return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(),
    // m_rightEncoder.getRate());

    // NEGATE RIGHT ENCODER VALUE!!!!!!!!!!!
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  // public void resetOdometry(Pose2d pose) {
  // resetEncoders();

  // m_odometry.resetPosition(pose, navX.getRotation2d());
  // }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    differentialDrive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotorControllerGroup.setVoltage(-leftVolts);
    rightMotorControllerGroup.setVoltage(-rightVolts);
    differentialDrive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public static void resetEncoders() {
    // m_leftEncoder.reset();
    // m_rightEncoder.reset();

    leftRelativeEncoder.setPosition(0);
    rightRelativeEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    // return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;

    // NEGATE RIGHT ENCODER VALUE!!!!!!!!!!!
    return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */

  // public Encoder getLeftEncoder() {
  // return m_leftEncoder;
  // }

  public RelativeEncoder getLeftEncoder() {
    return leftRelativeEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */

  // public Encoder getRightEncoder() {
  // return m_rightEncoder;
  // }

  public RelativeEncoder getRightEncoder() {
    return rightRelativeEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }

  // SPARK MAX will effectively short all motor wires together. This quickly
  // dissipates any electrical energy within the motor and brings it to a quick
  // stop.
  public void setBreakMode() {
    leftBackCANSparkMax.setIdleMode(IdleMode.kBrake);
    leftFrontCANSparkMax.setIdleMode(IdleMode.kBrake);
    rightFrontCANSparkMax.setIdleMode(IdleMode.kBrake);
    rightBackCANSparkMax.setIdleMode(IdleMode.kBrake);
  }

  public void setIdleMode() {
    leftBackCANSparkMax.setIdleMode(IdleMode.kCoast);
    leftFrontCANSparkMax.setIdleMode(IdleMode.kCoast);
    rightBackCANSparkMax.setIdleMode(IdleMode.kCoast);
    rightFrontCANSparkMax.setIdleMode(IdleMode.kCoast);
  }

  /** Zeroes the heading of the robot. */
  public static void zeroHeading() {
    navX.calibrate();
    navX.reset();

  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public static double getHeading() {
    // return 0;
    return navX.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -navX.getRate();
  }

}