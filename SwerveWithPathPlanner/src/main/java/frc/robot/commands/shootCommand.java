// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class shootCommand extends Command {
  private Kicker s_Kicker;
  private ShooterSubsystem s_ShooterSubsystem;
  private Superstructure superstructure;
  /** Creates a new shootCommand. */
  public shootCommand(Kicker s_Kicker, ShooterSubsystem s_ShooterSubsystem, Superstructure superstructure) {
    this.s_Kicker = s_Kicker;
    this.s_ShooterSubsystem = s_ShooterSubsystem;
    this.superstructure = superstructure;

    addRequirements(s_Kicker, s_ShooterSubsystem, superstructure);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_ShooterSubsystem.isReadyToFire() == true){
      s_Kicker.Kick(0.5);
      //superstructure.shooting = true;
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
