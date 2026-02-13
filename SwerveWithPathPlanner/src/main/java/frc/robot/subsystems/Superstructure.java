// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Utils.ShootingPhysics;
import frc.robot.Utils.ShootingPhysics.AimingSolution;
import frc.robot.Utils.ShootingTables;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.Supplier;


public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */

  private final TurretSubsystem turret;
  private final Shooter shooter;
  private final Kicker kicker;
  private final Spindexer spindexer;
  //private final ShooterSubsystem shooter;

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedSupplier;

  private final Translation2d TURRET_OFFSET = new Translation2d(0.28, -0.9);
  private final Translation2d BLUE_TARGET = new Translation2d(4.554, 4.068);
  private final Translation2d RED_TARGET = new Translation2d(11.9, 4);

  private final CommandXboxController operator;

  public boolean isTurretLockedOn = false;

  public boolean wantsIDLE = false;
  public boolean wantsShoot = false;

  //public boolean shooting = false;

  private final StructPublisher<Pose2d> turretTargetPub;

  private double operatorOffset = 0;

  public final double IDLERPM = 1500;

  public enum States{
    OFF,
    IDLE,
    SHOOTING
  }


  public Superstructure(TurretSubsystem turret, Shooter shooter, 
  Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier, 
  CommandXboxController operator, Kicker kicker, Spindexer spindexer) {
    this.turret = turret;
    this.poseSupplier = poseSupplier;
    this.speedSupplier = speedSupplier;
    this.operator = operator;
    this.shooter = shooter;
    this.kicker = kicker;
    this.spindexer = spindexer;

    
    var table = NetworkTableInstance.getDefault().getTable("Superstructure");
    turretTargetPub = table.getStructTopic("Turret", Pose2d.struct).publish();
  }

  public void toggleIdle(){
    wantsIDLE = !wantsIDLE;
  }

  public void toggleShooting(){
    wantsShoot = !wantsShoot;
  }

  /*public Superstructure(TurretSubsystem turret, ShooterSubsystem shooter, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
    this.turret = turret;
    this.shooter = shooter;
    this.poseSupplier = poseSupplier;
    this.speedSupplier = speedSupplier;
    
    var table = NetworkTableInstance.getDefault().getTable("Superstructure");
    turretTargetPub = table.getStructTopic("Turret", Pose2d.struct).publish();
  } */


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    States state = States.OFF;
    if(wantsShoot){
      state = States.OFF;
    }else if(wantsIDLE){
      state = States.SHOOTING;
    }else{
      state = States.OFF;
    }

    SmartDashboard.putString("Superstructure-state", state.toString());

    switch (state){
      case OFF:
        handleOFF();
        break;
      case IDLE:
        handleIDLE();
        break;
      case SHOOTING:
        handleSHOOTING();
        break;
    }


    Pose2d robotPose = poseSupplier.get();
    ChassisSpeeds robotSpeeds = speedSupplier.get();
    
    operatorOffset = operator.getLeftX();

    Translation2d currentTarget = BLUE_TARGET;
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent() && alliance.get() == Alliance.Red){
      currentTarget = RED_TARGET;
    }

    isTurretLockedOn = runAimingLoop(
      turret, shooter, robotPose, robotSpeeds, TURRET_OFFSET, currentTarget, "Turret", turretTargetPub
    );

    /* isTurretLockedOn = runAimingLoop(
      turret, shooter, robotPose, robotSpeeds, TURRET_OFFSET, currentTarget, "Turret", turretTargetPub
    ); */

    }
    public void handleOFF(){
      turret.setTargetAngle(new Rotation2d(0));
      shooter.stop();
      kicker.stop();
      kicker.stop();
    }

    public void handleIDLE(){
      AimingSolution solution = calculateAiming();
      turret.setTargetAngle(solution.turretAngle());

      shooter.setTargetRPM(false, IDLERPM);

      spindexer.stop();
      kicker.stop();
    }

    private void handleSHOOTING(){
      AimingSolution solution = calculateAiming();
      Rotation2d targetAngle = solution.turretAngle();
      if(Math.abs(operatorOffset) > 0.05) {
        targetAngle = targetAngle.plus(Rotation2d.fromDegrees(operatorOffset * 10));
        turret.setTargetAngle(targetAngle);
      } else {
        turret.setTargetAngle(targetAngle);
      }

      shooter.setTargetRPM(true, solution.effectiveDistance());

      boolean shooterReady = shooter.isReadyToFire();
      boolean turretLocked = Math.abs(turret.getErrorDegrees()) < 2.0;
      boolean locked = turretLocked && shooterReady;

      SmartDashboard.putBoolean("Superstructure-Locked", locked);

      if (locked) {
        kicker.Kick(0.5);
        if (spindexer.jammed) {
            spindexer.SpinCCW();
        } else {
            spindexer.SpinCW();
        }
    } else {
        kicker.stop();
        spindexer.stop();
    }
      
    }

    private AimingSolution calculateAiming(){
    Pose2d robotPose = poseSupplier.get();
    ChassisSpeeds robotSpeeds = speedSupplier.get();

    Translation2d currentTarget = BLUE_TARGET;
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent() && alliance.get() == Alliance.Red){
      currentTarget = RED_TARGET;
    }

    double rawDistance = robotPose.getTranslation().getDistance(currentTarget);
    double estimatedExitVel = ShootingTables.ExitVelocityMap.get(rawDistance);
    AimingSolution solution = ShootingPhysics.calculateAimingSolution(
        robotPose, robotSpeeds, TURRET_OFFSET, currentTarget, estimatedExitVel
    );

    turretTargetPub.set(solution.virtualTarget());
    
    return solution;
  }


    private boolean runAimingLoop(TurretSubsystem turret, Shooter shooter, Pose2d robotPose, ChassisSpeeds robotSpeeds, Translation2d offset, Translation2d targetLocation, String sideName, StructPublisher<Pose2d> publisher){
      double rawDistance = robotPose.getTranslation().getDistance(targetLocation);
      double estimatedExitVel = ShootingTables.ExitVelocityMap.get(rawDistance);
      AimingSolution solution = ShootingPhysics.calculateAimingSolution(robotPose, robotSpeeds, offset, targetLocation, estimatedExitVel);

      publisher.set(solution.virtualTarget());

      double targetRPM = ShootingTables.FlywheelMap.get(solution.effectiveDistance());

      

      //turret.setTargetAngle(solution.turretAngle());

      

      shooter.setTargetRPM(false, solution.effectiveDistance());
        
      

      boolean turretAtTarget = Math.abs(turret.getErrorDegrees()) < 2.0;
      boolean shooterAtSpeed = shooter.isReadyToFire();
      

      boolean locked = turretAtTarget && shooterAtSpeed;
      SmartDashboard.putBoolean(sideName + "/Locked", locked);

      SmartDashboard.putNumber(sideName + "/Aim/Dist_Effective", solution.effectiveDistance());
      SmartDashboard.putNumber(sideName + "/Aim/Target_Angle", solution.turretAngle().plus(Rotation2d.fromDegrees(operatorOffset * 10)).getDegrees());
      SmartDashboard.putNumber(sideName + "/Aim/RPM/_Top", targetRPM);

      return locked;
  }

  /*private boolean runAimingLoop(TurretSubsystem turret, ShooterSubsystem shooter, Pose2d robotPose, ChassisSpeeds robotSpeeds, Translation2d offset, Translation2d targetLocation, String sideName, StructPublisher<Pose2d> publisher){
      double rawDistance = robotPose.getTranslation().getDistance(targetLocation);
      double estimatedExitVel = ShootingTables.ExitVelocityMap.get(rawDistance);
      AimingSolution solution = ShootingPhysics.calculateAimingSolution(robotPose, robotSpeeds, offset, targetLocation, estimatedExitVel);

      publisher.set(solution.virtualTarget());

      double targetRPM = ShootingTables.FlywheelMap.get(solution.effectiveDistance());

      SmartDashboard.putNumber(sideName + "/Aim/Dist_Effective", solution.effectiveDistance());
      SmartDashboard.putNumber(sideName + "/Aim/Target_Angle", solution.turretAngle().getDegrees());
      SmartDashboard.putNumber(sideName + "/Aim/RPM/_Top", targetRPM);

      turret.setTargetAngle(solution.turretAngle());

      shooter.setTargetDistance(solution.effectiveDistance());

      boolean turretAtTarget = Math.abs(turret.getErrorDegrees()) < 2.0;
      boolean shooterAtSpeed = shooter.isReadyToFire();

      boolean locked = turretAtTarget && shooterAtSpeed;
      SmartDashboard.putBoolean(sideName + "/Locked", locked);

      return locked;
  } */

  public boolean canShoot(){return isTurretLockedOn;}
}
