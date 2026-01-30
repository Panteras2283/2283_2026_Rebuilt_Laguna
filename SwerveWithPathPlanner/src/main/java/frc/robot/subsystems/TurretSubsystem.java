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
import com.revrobotics.RelativeEncoder;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;



public class TurretSubsystem extends SubsystemBase {
  /** Creates a new Turret. */

  private final SparkFlex turretMotor;
  private final String Turret;
  private final RelativeEncoder turretEncoder;
  private final SparkClosedLoopController turretController;
  private final SparkFlexConfig turretConfig = new SparkFlexConfig();


  // Check with CAD
  private static final double GEAR_RATIO = 9;

  private static final double SOFT_LIMIT_FWD_ROT = 0.48;
  private static final double SOFT_LIMIT_BWD_ROT = -0.48;

  private static final double AIM_TOLERANCE_DEG = 2;

  private static final double kP = 10;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kFF = 12/380;

  private static final double maxVel_RPM = 650;
  private static final double maxAcc_RPMps = 750;

  public TurretSubsystem(int canId, String Turret) {
     this.Turret = Turret;

     this.turretMotor = new SparkFlex(canId, MotorType.kBrushless);
     this.turretEncoder = turretMotor.getEncoder();
     this.turretController = turretMotor.getClosedLoopController();

     configureTurret();
  }

  

  private void configureTurret(){
    SparkFlexConfig config = new SparkFlexConfig();

    config.encoder.positionConversionFactor(1.0/GEAR_RATIO);
    config.encoder.velocityConversionFactor(1.0/GEAR_RATIO);
    config.softLimit.forwardSoftLimitEnabled(true);
    config.softLimit.forwardSoftLimit(SOFT_LIMIT_FWD_ROT);
    config.softLimit.reverseSoftLimitEnabled(true);
    config.softLimit.reverseSoftLimit(SOFT_LIMIT_BWD_ROT);
    config.closedLoop.maxMotion.cruiseVelocity(maxVel_RPM);
    config.closedLoop.maxMotion.maxAcceleration(maxAcc_RPMps);
    config.closedLoop.maxMotion.allowedProfileError(0.008);
    config.closedLoop.feedForward.kV(kFF);
    config.closedLoop.pid(kP, kI, kD);
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(60);
    config.voltageCompensation(12.0);



    turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turretEncoder.setPosition(0);

  }

  public void setTargetAngle(Rotation2d targetAngle){
    double targetRotations = targetAngle.getRotations();

    targetRotations = MathUtil.inputModulus(targetRotations, -0.5, 0.5);
    targetRotations = MathUtil.clamp(targetRotations, SOFT_LIMIT_BWD_ROT, SOFT_LIMIT_FWD_ROT);

    turretController.setSetpoint(targetRotations, ControlType.kMAXMotionPositionControl);
  }

  public Rotation2d getCurrentAngle(){
    double rotations = turretEncoder.getPosition();
    return Rotation2d.fromRotations(rotations);
  }

  public double getErrorDegrees(){
    return 1.0;
  }

  public void resetPosition(){
    turretEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(Turret + "/AngleDeg", getCurrentAngle().getDegrees());
    SmartDashboard.putNumber(Turret + "/ErrorDeg", getErrorDegrees());
  }
}