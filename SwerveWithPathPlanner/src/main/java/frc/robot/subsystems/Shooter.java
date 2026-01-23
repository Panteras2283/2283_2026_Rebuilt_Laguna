// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.ShootingTables;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private final TalonFX Flywheel;

  private final VelocityVoltage flywheelRequest = new VelocityVoltage(0); 

  private String Turret;

  private double flywheelTargetRPM = 0;
  
  private static final double RPM_TOLERANCE_PERCENT = 0.03;

  public Shooter(int flywheelCanID, String Turret) {
    this.Turret = Turret;

    Flywheel = new TalonFX(flywheelCanID, "rio");
    configureKraken();
  }
  private void configureKraken(){
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.CurrentLimits.StatorCurrentLimit = 60.0;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    cfg.Slot0.kP = 0.1;
    cfg.Slot0.kV = 0.12;

    Flywheel.getConfigurator().apply(cfg);
    Flywheel.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setTargetDistance(double distanceMeters){
    flywheelTargetRPM = ShootingTables.FlywheelMap.get(distanceMeters);

    if(flywheelTargetRPM > 100){
      double targetRPS = flywheelTargetRPM / 60;
      Flywheel.setControl(flywheelRequest.withVelocity(targetRPS));
    } else{
      stop();
    }
  }

  public void stop(){
    Flywheel.stopMotor();
    flywheelTargetRPM = 0;
  }

  public boolean isReadyToFire(){
    double flywheelVelRPM = Flywheel.getVelocity().getValueAsDouble() * 60;

    double flywheelError = Math.abs(flywheelTargetRPM - flywheelVelRPM);

    boolean flywheelReady = flywheelError < (flywheelTargetRPM * RPM_TOLERANCE_PERCENT);

    return flywheelReady && (flywheelTargetRPM > 100);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  SmartDashboard.putNumber(Turret + "/FlywheelRPM", Flywheel.getVelocity().getValueAsDouble() * 60.0);
  SmartDashboard.putBoolean(Turret + "/Ready", isReadyToFire());
  }
}
