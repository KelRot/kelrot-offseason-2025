package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

public class climbModeSetup extends Command {
  private final ArmSubsystem m_ArmSubsystem;

  public climbModeSetup(ArmSubsystem m_ArmSubsystem) {
    this.m_ArmSubsystem = m_ArmSubsystem;
    addRequirements(m_ArmSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
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
    double angle = m_ArmSubsystem.getCurrentAngle();
    return angle >= 5;

  }
}
