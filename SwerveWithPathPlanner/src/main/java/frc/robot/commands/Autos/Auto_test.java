// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.Set;
import java.util.function.Supplier; // Import Supplier

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.commands.Fast_DriveToPose_cmd;
import frc.robot.commands.PID_Autopilot_cmd;
import frc.robot.commands.Precise_DriveToPose_cmd;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_test extends SequentialCommandGroup {

  /** Creates a new Auto_test. */
  public Auto_test(CommandSwerveDrivetrain drivetrain, PathConstraints constraints, Supplier<Integer> sideSupplier, Supplier<Integer> slotSupplier) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    /*  addCommands(

      new DeferredCommand(()-> {
        int targetSide = sideSupplier.get();
        int targetSlot = slotSupplier.get();

        Pose2d[][] targetPose = new Pose2d[3][0];
        Pose2d approachPose = targetPose.transformBy(new Transform2d(-0.25, 0.0, new Rotation2d()));

        return drivetrain.PathfindToPose(approachPose, constraints, 0.0);

      }, Set.of(drivetrain)),
       
      new DeferredCommand(()-> {
        int targetSide = sideSupplier.get();
        int targetSlot = slotSupplier.get();

        Pose2d[][] targetPose = new Pose2d[3][0];

        return new PID_Autopilot_cmd(drivetrain, targetPose, null, null);

      }, Set.of(drivetrain))
      ); */
  }
}
