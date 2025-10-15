package frc.robot.subsystems.climb;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climb extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax opener_motor, closer_motor; 
  private final SparkMaxConfig motorConfig;

  public Climb() {
    opener_motor = new SparkMax(Constants.ClimbConstants.openerNeoID, MotorType.kBrushless);
    closer_motor = new SparkMax(Constants.ClimbConstants.closerNeoID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    motorConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake);
    opener_motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    closer_motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void openClimb() {
    opener_motor.set(0.6);
  }
  public void closeClimb() {
    closer_motor.set(1);
    opener_motor.set(-0.6);
  }

  @Override
  public void periodic() { 
    if(SmartDashboard.getNumber("Climb/Opener Set", 0) != 0 || SmartDashboard.getNumber("Climb/Closer Set", 0) != 0) {
      setOpener();
      setCloser();
    }
  }

  @Override
  public void simulationPeriodic() {
  }

public void stopOpener() {
   opener_motor.set(0);
}
public void stopCloser() {
  closer_motor.set(0);
  opener_motor.set(0);
}
public void setOpener() {
  double num = SmartDashboard.getNumber("Climb/Opener Set", 0);
  opener_motor.set(num);
}
public void setOpener(double num) {
  opener_motor.set(num);
}
public void setCloser(double num) {
  closer_motor.set(num);
}
public void setCloser() {
  double num = SmartDashboard.getNumber("Climb/Closer Set", 0);
  closer_motor.set(num);
}
}
