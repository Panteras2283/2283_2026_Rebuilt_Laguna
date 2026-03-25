package frc.robot.commands;

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

public class Fast_DriveToPose_cmd extends SequentialCommandGroup {

    /**
     * Drives to a specific Reef location (dynamic or fixed) in two stages:
     * 1. Pathfinds to an "Approach Point" (1 meter away).
     * 2. Uses a PID Controller to drive the final meter precisely.
     * * @param drivetrain The drivetrain subsystem.
     * @param constraints The pathfinding constraints.
     * @param sideSupplier Supplier for the Reef Side (1-6).
     * @param slotSupplier Supplier for the Slot ID (0-2).
     * @param driver The driver controller (for rumble).
     * @param operator The operator controller (for rumble).
     */
    
    public Fast_DriveToPose_cmd(
            CommandSwerveDrivetrain drivetrain, 
            PathConstraints constraints,
            Supplier<Integer> sideSupplier, // Changed to Supplier
            Supplier<Integer> slotSupplier, // Changed to Supplier
            CommandXboxController driver, 
            CommandXboxController operator) {

        addCommands(
   
            new DeferredCommand(() -> {
                int targetSide = sideSupplier.get();
                int targetSlot = slotSupplier.get();

                Pose2d scoringPose = Constants.AutopilotConstants.getPose(targetSide, targetSlot);

                return drivetrain.PathfindToPose(scoringPose, constraints, 0.0);
                
            }, Set.of(drivetrain))
        );
    }
}