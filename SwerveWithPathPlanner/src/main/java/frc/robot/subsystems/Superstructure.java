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
  //private final ShooterSubsystem shooter;

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedSupplier;

  private final Translation2d TURRET_OFFSET = new Translation2d(0.28, -0.9);
  private final Translation2d BLUE_TARGET = new Translation2d(4.554, 4.068);
  private final Translation2d RED_TARGET = new Translation2d(11.9, 4);

  private final CommandXboxController operator = new CommandXboxController(1);

  private boolean isTurretLockedOn = false;

  private final StructPublisher<Pose2d> turretTargetPub;

  private double operatorOffset = 0;

  public Superstructure(TurretSubsystem turret, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
    this.turret = turret;
    this.poseSupplier = poseSupplier;
    this.speedSupplier = speedSupplier;
    
    var table = NetworkTableInstance.getDefault().getTable("Superstructure");
    turretTargetPub = table.getStructTopic("Turret", Pose2d.struct).publish();
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
    Pose2d robotPose = poseSupplier.get();
    ChassisSpeeds robotSpeeds = speedSupplier.get();

    operatorOffset = operator.getLeftX() * 10;

    Translation2d currentTarget = BLUE_TARGET;
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent() && alliance.get() == Alliance.Red){
      currentTarget = RED_TARGET;
    }

    isTurretLockedOn = runAimingLoop(
      turret, robotPose, robotSpeeds, TURRET_OFFSET, currentTarget, "Turret", turretTargetPub
    );

    /* isTurretLockedOn = runAimingLoop(
      turret, shooter, robotPose, robotSpeeds, TURRET_OFFSET, currentTarget, "Turret", turretTargetPub
    ); */

    }

    private boolean runAimingLoop(TurretSubsystem turret, Pose2d robotPose, ChassisSpeeds robotSpeeds, Translation2d offset, Translation2d targetLocation, String sideName, StructPublisher<Pose2d> publisher){
      double rawDistance = robotPose.getTranslation().getDistance(targetLocation);
      double estimatedExitVel = ShootingTables.ExitVelocityMap.get(rawDistance);
      AimingSolution solution = ShootingPhysics.calculateAimingSolution(robotPose, robotSpeeds, offset, targetLocation, estimatedExitVel);

      publisher.set(solution.virtualTarget());

      double targetRPM = ShootingTables.FlywheelMap.get(solution.effectiveDistance());

      SmartDashboard.putNumber(sideName + "/Aim/Dist_Effective", solution.effectiveDistance());
      SmartDashboard.putNumber(sideName + "/Aim/Target_Angle", solution.turretAngle().getDegrees());
      SmartDashboard.putNumber(sideName + "/Aim/RPM/_Top", targetRPM);

      //turret.setTargetAngle(solution.turretAngle());

      var adjustedAngle =
      solution.turretAngle().plus(
        Rotation2d.fromDegrees(operatorOffset)
      );

      turret.setTargetAngle(adjustedAngle);



      boolean turretAtTarget = Math.abs(turret.getErrorDegrees()) < 2.0;

      boolean locked = turretAtTarget;
      SmartDashboard.putBoolean(sideName + "/Locked", locked);

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
