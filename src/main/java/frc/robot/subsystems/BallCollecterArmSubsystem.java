// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.*;

public class BallCollecterArmSubsystem extends SubsystemBase {

  // figure out mechanism (limit switch, encoder, hall effect, etc)

  static WPI_TalonSRX ballCollecterArmTalonSRX = new WPI_TalonSRX(
      Constants.CollecterConstants.ballCollecterArmTalonSRX);

  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final static I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a
   * parameter. The device will be automatically initialized with default
   * parameters.
   */
  private final static ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This
   * can
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final static ColorMatch m_colorMatcher = new ColorMatch();

  public ColorMatch getColorMatcher() {
    return m_colorMatcher;
  }

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  private final static Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final static Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final static Color kRedTarget = new Color(0.561, 0.232, 0.114);

  // private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

  /** Creates a new ColorSensorSubsystem. */

  public static ColorSensorV3 getColorSensorV3() {
    return m_colorSensor;
  }

  public static WPI_TalonSRX getBallCollecterArmTalonSRX() {
    return ballCollecterArmTalonSRX;
  }

  public void setCollecterArmSpeed(double speed) {
    ballCollecterArmTalonSRX.set(speed);
  }

  public static String detectColor() {
    /**
     * The method GetColor() returns a normalized color value from the sensor and
     * can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the
     * measurements and make it difficult to accurately determine its color.
     */
    Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else {
      colorString = "Unknown";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);

    SmartDashboard.putString("Detected Color", colorString);
    return colorString;
  }

  public BooleanSupplier isSameColor() {
    String ballColor = detectColor();
    if (ballColor.equals(RobotContainer.getAllianceColor())) {
      System.out.println("same color!");
      return () -> true;
    }
    return () -> false;

  }

  /** Creates a new BallCollecterArm. */
  public BallCollecterArmSubsystem() {

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    // m_colorMatcher.addColorMatch(kYellowTarget);

    ballCollecterArmTalonSRX.setSelectedSensorPosition(0);

    /* Factory default hardware to prevent unexpected behavior */
    ballCollecterArmTalonSRX.configFactoryDefault();

    /* Configure Sensor Source for Pirmary PID */
    ballCollecterArmTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
        Constants.CollecterArmConstants.kPIDLoopIdx,
        Constants.CollecterArmConstants.kTimeoutMs);

    /*
     * set deadband to super small 0.001 (0.1 %).
     * The default deadband is 0.04 (4 %)
     */
    ballCollecterArmTalonSRX.configNeutralDeadband(0.001, Constants.CollecterArmConstants.kTimeoutMs);

    /**
     * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
     * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
     * sensor to have positive increment when driving Talon Forward (Green LED)
     */
    ballCollecterArmTalonSRX.setSensorPhase(false);
    ballCollecterArmTalonSRX.setInverted(false);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    ballCollecterArmTalonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10,
        Constants.CollecterArmConstants.kTimeoutMs);
    ballCollecterArmTalonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10,
        Constants.CollecterArmConstants.kTimeoutMs);

    /* Set the peak and nominal outputs */
    ballCollecterArmTalonSRX.configNominalOutputForward(0, Constants.CollecterArmConstants.kTimeoutMs);
    ballCollecterArmTalonSRX.configNominalOutputReverse(0, Constants.CollecterArmConstants.kTimeoutMs);
    ballCollecterArmTalonSRX.configPeakOutputForward(1, Constants.CollecterArmConstants.kTimeoutMs);
    ballCollecterArmTalonSRX.configPeakOutputReverse(-1, Constants.CollecterArmConstants.kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
    ballCollecterArmTalonSRX.selectProfileSlot(Constants.CollecterArmConstants.kSlotIdx,
        Constants.CollecterArmConstants.kPIDLoopIdx);
    ballCollecterArmTalonSRX.config_kF(Constants.CollecterArmConstants.kSlotIdx,
        Constants.CollecterArmConstants.kGains.kF, Constants.CollecterArmConstants.kTimeoutMs);
    ballCollecterArmTalonSRX.config_kP(Constants.CollecterArmConstants.kSlotIdx,
        Constants.CollecterArmConstants.kGains.kP, Constants.CollecterArmConstants.kTimeoutMs);
    ballCollecterArmTalonSRX.config_kI(Constants.CollecterArmConstants.kSlotIdx,
        Constants.CollecterArmConstants.kGains.kI, Constants.CollecterArmConstants.kTimeoutMs);
    ballCollecterArmTalonSRX.config_kD(Constants.CollecterArmConstants.kSlotIdx,
        Constants.CollecterArmConstants.kGains.kD, Constants.CollecterArmConstants.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    ballCollecterArmTalonSRX.configMotionCruiseVelocity(973, Constants.CollecterArmConstants.kTimeoutMs);
    ballCollecterArmTalonSRX.configMotionAcceleration(972.75, Constants.CollecterArmConstants.kTimeoutMs);

    ballCollecterArmTalonSRX.configFeedbackNotContinuous(true, Constants.CollecterArmConstants.kTimeoutMs);

    // Configure current limits
    ballCollecterArmTalonSRX.configPeakCurrentLimit(30);
    ballCollecterArmTalonSRX.configPeakCurrentDuration(150);

    // takes in AMPS
    ballCollecterArmTalonSRX.configContinuousCurrentLimit(20);

    // integral zone
    ballCollecterArmTalonSRX.config_IntegralZone(Constants.CollecterArmConstants.kSlotIdx, 3);

  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("talon encoder value", ballCollecterArmTalonSRX.getSelectedSensorPosition());

    // if (ballCollecterArmTalonSRX.getControlMode() == ControlMode.MotionMagic
    // && ballCollecterArmTalonSRX.getSelectedSensorPosition() > 2000) {
    // ballCollecterArmTalonSRX.setNeutralMode(NeutralMode.Brake);
    // } else {
    // ballCollecterArmTalonSRX.setNeutralMode(NeutralMode.Coast);
    // }

    SmartDashboard.putString("talon mode", ballCollecterArmTalonSRX.getControlMode().toString());
    SmartDashboard.putNumber("output", ballCollecterArmTalonSRX.getMotorOutputPercent());

    detectColor();

    // This method will be called once per scheduler run

    // if (colorString.equals("Green")) {
    // System.out.println("do nothing!");
    // } else

  }
}
