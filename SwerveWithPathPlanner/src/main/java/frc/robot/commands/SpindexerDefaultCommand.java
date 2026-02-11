// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpindexerDefaultCommand extends Command {
  private Spindexer s_Spindexer;
  private Intake s_Intake;
  private Superstructure s_Superstructure;
  /** Creates a new SpindexerDefaultCommand. */
  public SpindexerDefaultCommand(Spindexer s_Spindexer, Intake s_Intake, Superstructure s_Superstructure) {
    this.s_Intake = s_Intake;
    this.s_Spindexer = s_Spindexer;
    this.s_Superstructure = s_Superstructure;
    addRequirements(s_Intake, s_Spindexer, s_Superstructure);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_Intake.feeding == true || s_Spindexer.jammed == true){
      new AntijamCommand(s_Spindexer);
    }else if(s_Superstructure.shooting == true){
      s_Spindexer.SpinCW();
    }else if(s_Intake.outake == true){
      s_Spindexer.SpinCCW();
    }else{
      s_Spindexer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
