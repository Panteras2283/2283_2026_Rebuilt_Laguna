// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Spindexer extends SubsystemBase {

  private TalonFX Spindexer = new TalonFX(Constants.Spindexer.motorID);
  private TalonFX RollerMotor = new TalonFX(Constants.Kicker.rollerID);
  /** Creates a new Spindexer. */
  public Spindexer() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void SpinWI(double speed){
    Spindexer.set(speed);
    RollerMotor.set(-speed);
  }

  public void stop(){
    Spindexer.set(0);
    RollerMotor.set(0);
  }
}
