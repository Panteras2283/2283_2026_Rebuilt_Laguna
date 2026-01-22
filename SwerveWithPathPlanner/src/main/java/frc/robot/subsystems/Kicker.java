// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {

  private TalonFX KickerMotor = new TalonFX(Constants.Kicker.motorID);
  private TalonFX RollerMotor = new TalonFX(Constants.Kicker.rollerID);
  private TalonFX Spindexer = new TalonFX(Constants.Spindexer.motorID);
  /** Creates a new Kicker. */
  public Kicker() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Kick(double speed){
    KickerMotor.set(speed);
    RollerMotor.set(speed);
    Spindexer.set(speed*0.7);
  }

  public void stop(){
    KickerMotor.set(0);
    RollerMotor.set(0);
    Spindexer.set(0);
  }
}
