// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.Constants;


public class Intake extends SubsystemBase {
 /** Creates a new Intake_demo. */
  private TalonFX Feeder = new TalonFX(Constants.Intake.FeederID);
  private TalonFX pivotLeft = new TalonFX(Constants.Intake.PivotLeftID);
  private TalonFX pivotRight = new TalonFX(Constants.Intake.PivotRightID);

  private final MotionMagicVoltage leftPivotRequest = new MotionMagicVoltage(0);
  private final MotionMagicVoltage rightPivotRequest = new MotionMagicVoltage(0);
  private final VelocityVoltage feederRequest = new VelocityVoltage(0); 

  public boolean feeding = false;

  public boolean outake = false;
  
  public Intake() {
    configureMotionMagic();
    configureFeeder();
  }

  private void configureFeeder(){
    TalonFXConfiguration cfgF = new TalonFXConfiguration();
    cfgF.CurrentLimits.StatorCurrentLimit = 100.0;
    cfgF.CurrentLimits.StatorCurrentLimitEnable = true;
    cfgF.Slot0.kP = 0.33;
    cfgF.Slot0.kV = 0.12;

    Feeder.getConfigurator().apply(cfgF);
    Feeder.setNeutralMode(NeutralModeValue.Coast);
    }

  private void configureMotionMagic(){
    TalonFXConfiguration cfgMm = new TalonFXConfiguration();

    cfgMm.Slot0.kP = 1.0;   
    cfgMm.Slot0.kI = 0.0;
    cfgMm.Slot0.kD = 0.0;
    cfgMm.Slot0.kV = 0.0;   
    cfgMm.Slot0.kS = 0.0;

    cfgMm.MotionMagic.MotionMagicCruiseVelocity = 20;
    cfgMm.MotionMagic.MotionMagicAcceleration = 40;
    cfgMm.MotionMagic.MotionMagicJerk = 0;

    cfgMm.CurrentLimits.StatorCurrentLimit = 60.0;
    cfgMm.CurrentLimits.StatorCurrentLimitEnable = true;

    pivotLeft.getConfigurator().apply(cfgMm);
    pivotRight.getConfigurator().apply(cfgMm);

    pivotLeft.setNeutralMode(NeutralModeValue.Brake);
    pivotRight.setNeutralMode(NeutralModeValue.Brake);

    pivotLeft.setPosition(0);
    pivotRight.setPosition(0);
  }
    
  @Override
  public void periodic() {
    SmartDashboard.putNumber("LeftPivotPos", pivotLeft.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("RightPivotPos", pivotRight.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("FeederVel", Feeder.getVelocity().getValueAsDouble());
  }

  public void Down(){
    pivotLeft.setControl(leftPivotRequest.withPosition(Constants.Intake.LeftFeedPos));
    pivotRight.setControl(rightPivotRequest.withPosition(Constants.Intake.RightFeedPos));
  }  

  public void feedOnly(){
    feeding = true;
    Feeder.setControl(feederRequest.withVelocity(100));
    
  } 

  public void up(){
    pivotLeft.setControl(leftPivotRequest.withPosition(Constants.Intake.LeftUpPos));
    pivotRight.setControl(rightPivotRequest.withPosition(Constants.Intake.RightUpPos));
  }

  public void outake(){
    outake = true;
    Feeder.set(-0.5);
    
  } 

  public void stop(){
    outake = false;
    Feeder.set(0);
    feeding = false;
  }
}
