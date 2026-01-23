// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */

  private final TalonFX turretMotor;
  private final String Turret;

  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

  // Check with CAD
  private static final double GEAR_RATIO = 15.0;

  private static final double SOFT_LIMIT_FWD_ROT = 0.48;
  private static final double SOFT_LIMIT_BWD_ROT = -0.48;

  private static final double AIM_TOLERANCE_DEG = 2.0;

  public Turret(int canId, String Turret) {
     this.Turret = Turret;

     this.turretMotor = new TalonFX(canId, "rio");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
