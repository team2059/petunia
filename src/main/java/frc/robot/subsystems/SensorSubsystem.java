// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.I2C;

public class SensorSubsystem extends SubsystemBase {

  /*
    TODO - Color Sensor (Calibrate values)
    TODO - Light Sensor on Floor (2 Sensors if...)
    TODO - Light Sensor on Ball Elevator/Collector 
  */ 

  //TODO - Change port numbers
  DigitalInput ballPhotoElectricSensor = new DigitalInput(1);
  DigitalInput groundPhotoElectricSensor = new DigitalInput(2);

  I2C.Port i2cColorSensorPort = I2C.Port.kOnboard;

  ColorSensorV3 ballColorSensor = new ColorSensorV3(i2cColorSensorPort);

  ColorMatch colorMatcher = new ColorMatch();

  String colorString = "";
  
  private final static Color kBlueTarget = new Color(0.143, 0.427, 0.429); 
  private final static Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final static Color kRedTarget = new Color(0.561, 0.232, 0.114);

  /** Creates a new SensorSubsystem. */
  public SensorSubsystem() {
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
  }
  
  public boolean isBallInChamber(){
    return ballPhotoElectricSensor.get();
  }
  
//TODO make a method that processes line color changes

  public boolean findBlackTape(){
    return groundPhotoElectricSensor.get();
  }

  public String detectColor() {
    
    Color detectedColor = ballColorSensor.getColor();

    ColorMatchResult colorMatch = colorMatcher.matchClosestColor(detectedColor);

    if (colorMatch.color == kBlueTarget) {
      colorString = "Blue";
    } else if (colorMatch.color == kGreenTarget) {
      colorString = "Green";
    } else if (colorMatch.color == kRedTarget) {
      colorString = "Red";
    } 

    return colorString;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
