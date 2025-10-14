package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LedConstants;
import frc.robot.subsystems.leds.LedSubsystem;

public class RobotStatusManager {
    public enum RobotStatus {
        Alignment(255,0,0, true, false), // Blinking Red
        CountDown_Intake(0, 255, 0, true, false), // Blinking Green
        Has_Coral(0,255,255, false, false), // Solid Cyan
        ShootingL1(255, 0, 255, true, false),     // Blinking Purple
        ShootingL2(247, 180, 198, true, false),       // Blinking Pink
        ShootingL3(255, 0, 0, true, false),       // Blinking Red
        Removing_Algae(0, 255, 0, true, false),    // Blinking Green // Blinking Green
        Climbing(0, 0, 0, false, true),          // Rainbow
        Default(0, 0, 0, false, false);           // Pitch Black 
        public final int r, g, b;
        public final boolean blinking;
        public final boolean rainbow;

        RobotStatus(int r, int g, int b, boolean blinking, boolean rainbow) {
            this.r = r;
            this.g = g;
            this.b = b;
            this.blinking = blinking;
            this.rainbow = rainbow;
        }
    }
    
    private RobotStatus currentStatus;
    private final LedSubsystem m_led;

    public RobotStatusManager(LedSubsystem ledSubsystem) {
       this.currentStatus = RobotStatus.Default;
       this.m_led = ledSubsystem;
    }

    public void setStatus(RobotStatus newStatus) {
        if (this.currentStatus != newStatus) { 
            this.currentStatus = newStatus;
            SmartDashboard.putString("dev/Robot Status", newStatus.name());
            updateLedStatus();
        }
    }

    public RobotStatus getStatus() {
        return this.currentStatus;
    }

    public void updateLedStatus(){
        RobotStatus status = getStatus();

        if (status.rainbow) {
            m_led.rainbow(new int[]{0});
        } else if (status.blinking) {
            m_led.setBlinkColor(new edu.wpi.first.wpilibj.util.Color(status.r, status.g, status.b), new int[]{0});
        } else {
            m_led.setSolidColor(new edu.wpi.first.wpilibj.util.Color(status.r, status.g, status.b), new int[]{0});
        }
    }

    public void periodic() {
    }
}

