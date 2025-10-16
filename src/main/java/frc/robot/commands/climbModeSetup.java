package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.ArmSubsystem;

public class climbModeSetup extends Command {
  private final ArmSubsystem m_ArmSubsystem;
  private Trigger climbFinishedTrigger;

  public climbModeSetup(ArmSubsystem m_ArmSubsystem) {
    this.m_ArmSubsystem = m_ArmSubsystem;
    this.climbFinishedTrigger = new Trigger(() -> {
      double angle = m_ArmSubsystem.getCurrentAngle();
      return angle >= 5 && angle <= 15;
    });
    addRequirements(m_ArmSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.m_ArmSubsystem.setSetpoint(10);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_ArmSubsystem.setMode(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climbFinishedTrigger.getAsBoolean();
  }
}
