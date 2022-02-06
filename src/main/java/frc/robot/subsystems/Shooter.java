// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
public class Shooter extends SubsystemBase {
  //Change constant 
  WPI_TalonSRX ballShooter = new WPI_TalonSRX(0);
  WPI_TalonSRX indexMotor = new WPI_TalonSRX(0);
  DigitalInput testHFX = new DigitalInput(0);
  /** Creates a new Shooter. */
  public Shooter() {
    ballShooter.configFactoryDefault();
    ballShooter.setSensorPhase(true);
    //set constant timeout
    ballShooter.configNominalOutputForward(0,30);
    ballShooter.configNominalOutputReverse(0,30);
    ballShooter.configPeakOutputForward(1,30);
    ballShooter.configPeakOutputReverse(-1,30);
    ballShooter.selectProfileSlot(0, 0);
    
    indexMotor.configFactoryDefault();
    indexMotor.setSensorPhase(true);
    //set constant timeout
    indexMotor.configNominalOutputForward(0,30);
    indexMotor.configNominalOutputReverse(0,30);
    indexMotor.configPeakOutputForward(1,30);
    indexMotor.configPeakOutputReverse(-1,30);
    indexMotor.selectProfileSlot(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Ball Shooter Velocity",ballShooter.getSelectedSensorVelocity());
  }
  public double getShooterVelocity(){
    return ballShooter.getSelectedSensorVelocity();
  }
  public void setShooterVelocity(){

  }
}