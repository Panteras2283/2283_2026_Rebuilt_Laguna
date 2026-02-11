// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.Constants;


public class Intake extends SubsystemBase {
 /** Creates a new Intake_demo. */
  private TalonFX Feeder = new TalonFX(Constants.Intake.FeederID);

  public boolean feeding = false;

  public boolean outake = false;
  
  public Intake() {}

  @Override
  public void periodic() {
    
  
  }

  public void feed(){
    outake = false;
    feeding = true;
    Feeder.set(0.5);
    
  } 

  public void outake(){
    outake = true;
    feeding = false;
    Feeder.set(-0.5);
    
  } 

  public void stop(){
    outake = false;
    Feeder.set(0);
    feeding = false;
  }
}
