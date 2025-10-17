package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.RobotStatusManager;
import frc.robot.subsystems.RobotStatusManager.RobotStatus;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.wrist.wristSubsystem;

public class IntakeCmd extends Command {
    private final ArmSubsystem m_arm;
    private final wristSubsystem m_wrist;
    private final Timer m_timer;
    private boolean timeron, m_finished;

    public IntakeCmd(ArmSubsystem arm, wristSubsystem wrist) {
        m_timer = new Timer();
        m_arm = arm;
        m_wrist = wrist;
        timeron = false;
        m_finished = false;
        addRequirements(arm, wrist);
    }

    @Override
    public void execute() {
        m_arm.setSetpoint(Constants.LevelAngles.DefaultAngle);
        m_wrist.setSetpoint(Constants.LevelAngles.DefaultAngleWrist);
        if (m_wrist.getSensor()) {
            m_finished = false;
            m_wrist.setWheelMotor(5.3);
        } else {
            if (!timeron) {
                m_timer.reset();
                m_timer.start();
                timeron = true;
            } else if (m_timer.get() > 0.12) {
                m_wrist.setWheelMotor(0);
                m_finished = true;
            }
        }

    }

    @Override
    public void end(boolean isFinished) {
        m_wrist.setWheelMotor(0);
        timeron = false;
        m_finished = false;
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
}