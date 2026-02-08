// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Spindexer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AntijamCommand extends SequentialCommandGroup {
  private Spindexer s_Spindexer;

  /** Creates a new AntijamCommand. */
  public AntijamCommand(Spindexer s_Spindexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.s_Spindexer = s_Spindexer;
    addRequirements(s_Spindexer);
    addCommands(
      new InstantCommand(()-> s_Spindexer.SpinCW()),
      new WaitCommand(0.5),
      new InstantCommand(()-> s_Spindexer.SpinCCW()),
      new WaitCommand(0.5)
    );
  }
}
