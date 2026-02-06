// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Spindexer extends SubsystemBase {

  private TalonFX SpindexerMotor = new TalonFX(Constants.Spindexer.motorID);

  private VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  public final StatusSignal<Voltage> SpindexerVoltage = SpindexerMotor.getMotorVoltage();
 

  /** Creates a new Spindexer. */
  public Spindexer() {

    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.Spindexer.slot0P;
    slot0Configs.kI = Constants.Spindexer.slot0I;
    slot0Configs.kD = Constants.Spindexer.slot0D;
    slot0Configs.kS = Constants.Spindexer.slot0S;
    slot0Configs.kV = Constants.Spindexer.slot0V;
    slot0Configs.kA = Constants.Spindexer.slot0A;

    SpindexerMotor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double SpindexerRPM = SpindexerMotor.getVelocity().getValueAsDouble() * 60;
    
    double volts = SpindexerVoltage.getValueAsDouble();
    SpindexerVoltage.refresh();
    

    if (volts > 12 && SpindexerRPM < 6000){
      SpindexerMotor.setControl(m_request.withVelocity(-100));
    } else{

    }
  }

  public void SpinWI(){
    SpindexerMotor.setControl(m_request.withVelocity(100));

  }

  public void stop(){
    SpindexerMotor.set(0);
  }
}
