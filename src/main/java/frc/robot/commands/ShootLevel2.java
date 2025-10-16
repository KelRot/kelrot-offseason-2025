package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.RobotStatusManager;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.wrist.wristSubsystem;

public class ShootLevel2 extends SequentialCommandGroup {

  private final ArmSubsystem m_ArmSubsystem;
  private final wristSubsystem m_wristsub;
  public ShootLevel2(ArmSubsystem m_ArmSubsystem, wristSubsystem m_wrist, RobotStatusManager m_robotStatusManager) {

    this.m_wristsub = m_wrist;
    this.m_ArmSubsystem = m_ArmSubsystem;
    addRequirements(m_ArmSubsystem, m_wrist);

    addCommands(
    new InstantCommand(() -> this.m_ArmSubsystem.setSetpoint(Constants.LevelAngles.Level2)),
    new WaitCommand(0.4),
    new InstantCommand(() -> this.m_wristsub.setWheelMotor(9)),
    new WaitCommand(0.7),
    new InstantCommand(() -> this.m_wristsub.setWheelMotor(0)),
    new InstantCommand(() -> this.m_ArmSubsystem.setSetpoint(Constants.LevelAngles.DefaultAngle))
    );

  }
}
