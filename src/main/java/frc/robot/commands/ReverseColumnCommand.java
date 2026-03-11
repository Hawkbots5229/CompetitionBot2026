// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReverseColumnCommand extends Command {
  /** Creates a new ReverseColumnCommand. */
  private final Feeder s_feeder;
  private final Floor s_floor;
  private final Shooter s_shooter;
  
  public ReverseColumnCommand(Feeder s_feeder, Floor s_floor, Shooter s_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_feeder, s_floor, s_shooter);
    this.s_feeder = s_feeder;
    this.s_floor = s_floor;
    this.s_shooter = s_shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_feeder.setPercentOutput(-1.0);
    s_floor.setPercentOutput(-1.0);
    s_shooter.setPercentOutput(-1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_feeder.setPercentOutput(0.0);
    s_floor.setPercentOutput(0.0);
    s_shooter.setPercentOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
