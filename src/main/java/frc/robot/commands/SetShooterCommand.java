// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetShooterCommand extends Command {

  private final Shooter s_shooter;

  /** Creates a new SetShooterCommand. */
  public SetShooterCommand(Shooter s_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_shooter);
    this.s_shooter = s_shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joyOffset = RobotContainer.mech.getRightY()*100;
    double newTarget = s_shooter.getDashboardTargetRPM()-joyOffset;

    if(Math.abs(joyOffset) > 1) {
        s_shooter.setDashboardTargetRPM(newTarget);
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
