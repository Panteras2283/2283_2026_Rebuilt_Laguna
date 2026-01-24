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

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new Turret. */

  private final TalonFX turretMotor;
  private final String Turret;

  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

  // Check with CAD
  private static final double GEAR_RATIO = 15.0;

  private static final double SOFT_LIMIT_FWD_ROT = 0.48;
  private static final double SOFT_LIMIT_BWD_ROT = -0.48;

  private static final double AIM_TOLERANCE_DEG = 2.0;

  public TurretSubsystem(int canId, String Turret) {
     this.Turret = Turret;

     this.turretMotor = new TalonFX(canId, "rio");

     configureTurret();
  }

  private void configureTurret(){
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = SOFT_LIMIT_FWD_ROT;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = SOFT_LIMIT_BWD_ROT;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    cfg.MotionMagic.MotionMagicCruiseVelocity = 2.5;
    cfg.MotionMagic.MotionMagicAcceleration = 5.0;
    cfg.MotionMagic.MotionMagicJerk = 0.0;

    cfg.Slot0.kP = 40.0;
    cfg.Slot0.kI = 0;
    cfg.Slot0.kD = 0.5;
    cfg.Slot0.kV = 0.12;
    cfg.Slot0.kS = 0.25;

    cfg.CurrentLimits.StatorCurrentLimit = 60.0;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    turretMotor.getConfigurator().apply(cfg);

    turretMotor.setNeutralMode(NeutralModeValue.Brake);

    turretMotor.setPosition(0);

  }

  public void setTargetAngle(Rotation2d targetAngle){
    double targetRotations = targetAngle.getRotations();

    targetRotations = MathUtil.inputModulus(targetRotations, -0.5, 0.5);

    targetRotations = MathUtil.clamp(targetRotations, SOFT_LIMIT_BWD_ROT, SOFT_LIMIT_FWD_ROT);

    turretMotor.setControl(mmRequest.withPosition(targetRotations));
  }

  public Rotation2d getCurrentAngle(){
    double rotations = turretMotor.getPosition().getValueAsDouble();
    return Rotation2d.fromRotations(rotations);
  }

  public double getErrorDegrees(){
    double errorRotations = turretMotor.getClosedLoopError().getValueAsDouble();
    return Rotation2d.fromRotations(errorRotations).getDegrees();
  }

  public void resetPosition(){
    turretMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(Turret + "/AngleDeg", getCurrentAngle().getDegrees());
    SmartDashboard.putNumber(Turret + "/ErrorDeg", getErrorDegrees());
  }
}