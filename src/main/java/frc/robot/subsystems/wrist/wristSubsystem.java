package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.kelrotlib.utils.TunableNumber;
import frc.robot.Constants;

public class wristSubsystem extends SubsystemBase {
  private TalonFX masterMotor;
  private WPI_VictorSPX m_wheelMotor;
  private final DigitalInput m_sensor;
  private TunableNumber kP = new TunableNumber("Wrist/kP", 0.03);
  private TunableNumber kI = new TunableNumber("Wrist/kI", 0.0);
  private TunableNumber kD = new TunableNumber("Wrist/kD", 0.00000098);
  private TunableNumber debugSetpoint = new TunableNumber("Wrist/debugSetPoint", 0);
  private PIDController pidController;
  private double setPoint;

  /** Creates a new ExampleSubsystem. */
  public wristSubsystem() {
    masterMotor = new TalonFX(Constants.WristConstants.FalconID, "rio");
    m_wheelMotor = new WPI_VictorSPX(Constants.WristConstants.wheelMotorID);
    m_sensor = new DigitalInput(2);
    pidController = new PIDController(kP.lastValue, kI.lastValue, kD.lastValue);
    pidController.setTolerance(0);
  }

  public void setVoltage() {
    double num = SmartDashboard.getNumber("Wrist/Debug Voltage", 0);
    masterMotor.setVoltage(num);
  }

  public void setVoltage(double num) {
    masterMotor.setVoltage(num);
  }

  public void setWheelMotor(double volts) {
    m_wheelMotor.setVoltage(volts);
  }

  public boolean getSensor() {
    return m_sensor.get();
  }

  public double getAngle() {
    return Rotations.of(masterMotor.getPosition().getValueAsDouble()).in(Degrees) / 8.125 + 15;
  }

  public double getRealAngle() {
    return -(getAngle() + -13) - getAngle() - 93;
  }

  public void reachSetPoint(double setPoint) {
    if (SmartDashboard.getBoolean("dev/debugMode", false)) {
      // Implement PID control logic here to reach the desired setPoint
      if (SmartDashboard.getBoolean("dev/isRioPIDController", true)) {
        double pidOutput = pidController.calculate(getRealAngle(), setPoint);
        double ffOutput = getFeedForward(setPoint);
        masterMotor.setVoltage(pidOutput + ffOutput);
      } else {
        // will add talon fx closed loop later
      }
    }
  }

  public void setSetpoint(double setPoint) {
    this.setPoint = setPoint;
  }

  public double getSetpoint() {
    return this.setPoint;
  }

  public double getFeedForward(double angleInDegrees) { // Calculates The Feed Forward Value
    double direction = angleInDegrees < 0 ? -1 : 1;
    return direction * (1.02 * Math.abs(Math.sin(Math.toRadians(Math.abs(angleInDegrees)))));
  }

  public Command reachSetPointCommand() {
    return run(() -> reachSetPoint(getSetpoint()));
  }
  public Command TreachSetPointCommand() {
    return run(() -> reachSetPoint(debugSetpoint.lastValue));
  }

  @Override
  public void periodic() {
    if (SmartDashboard.getBoolean("dev/debugMode", false)) {
      setVoltage();
    }
    pidController.setP(kP.lastValue);
    pidController.setI(kI.lastValue);
    pidController.setD(kD.lastValue);
    SmartDashboard.putNumber("Wrist/Angle", getRealAngle());
    SmartDashboard.putNumber("Wrist/Setpoint", getSetpoint());
    SmartDashboard.putNumber("Wrist/PID Output", pidController.calculate(getRealAngle(), setPoint));
    SmartDashboard.putNumber("Wrist/Feed Forward", getFeedForward(setPoint));
    SmartDashboard.putBoolean("Wrist/Bottom Sensor", getSensor());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
