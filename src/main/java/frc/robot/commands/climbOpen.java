package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class climbOpen extends Command {
  private final Climb m_Climb;

  public climbOpen(Climb m_Climb) {
    this.m_Climb = m_Climb;
    addRequirements(m_Climb);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.m_Climb.openClimb();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_Climb.stopOpener();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
