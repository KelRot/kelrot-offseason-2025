package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.kelrotlib.utils.TunableNumber;

public class ArmSubsystem extends SubsystemBase {
  public TunableNumber kP = new TunableNumber("Arm/kP", 0.034);
  public TunableNumber kI = new TunableNumber("Arm/kI", 0.0);
  public TunableNumber kD = new TunableNumber("Arm/kD", 0.00000098);
  public SparkMax masterMotor, slaveMotor;
  public double setPoint = 0.0;
  public SparkMaxConfig masterConfig, slaveConfig;
  private final SparkClosedLoopController closedLoopController;
  private final Encoder quadEncoder;
  private boolean isStopped = false;
  private PIDController pidController = new PIDController(kP.lastValue, kI.lastValue, kD.lastValue);

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    this.quadEncoder = new Encoder(0, 1); // Replace 0, 1 with actual ports
    this.masterMotor = new SparkMax(Constants.ArmConstants.MASTER_MOTOR_ID, SparkMax.MotorType.kBrushless);
    this.slaveMotor = new SparkMax(Constants.ArmConstants.SLAVE_MOTOR_ID, SparkMax.MotorType.kBrushless);
    this.closedLoopController = masterMotor.getClosedLoopController();
    configureMotors();
    configureEncoder();
  }

  public double calculateFF(double angleInDegrees) {
    double direction = angleInDegrees > 0 ? -1 : 1;
    return direction * (0.96 * Math.abs(Math.sin(Math.toRadians(Math.abs(angleInDegrees)))));
  }

  public void reachSetPoint(double setPoint) {
    // Implement PID control logic here to reach the desired setPoint
    if (SmartDashboard.getBoolean("dev/isRioPIDController", true)) {
      double pidOutput = pidController.calculate(getCurrentAngle(), setPoint);
      double ffOutput = calculateFF(getCurrentAngle());
      masterMotor.setVoltage(pidOutput + ffOutput);
    } else {
      this.closedLoopController.setReference(setPoint, ControlType.kPosition, ClosedLoopSlot.kSlot0,
          calculateFF(getCurrentAngle()));
    }

  }

  public double getCurrentAngle() {
    // Convert encoder counts to angle in degrees
    return (quadEncoder.getDistance() - Constants.ArmConstants.defaultAngle);
  }

  public void setSetpoint(double setPoint) {
    this.setPoint = setPoint;
  }

  public double getSetpoint() {
    return this.setPoint;
  }

  public void setVoltage() {
    TunableNumber debugVoltage = new TunableNumber("Arm/Voltage", 0.0);
    masterMotor.setVoltage(debugVoltage.lastValue);
  }

  public void setMode(boolean isStopped) {
    this.isStopped = isStopped;

  }

  @Override
  public void periodic() {
    if (this.isStopped) {
      masterMotor.setVoltage(0);
    } else {
      if (SmartDashboard.getBoolean("dev/debugMode", false)) {
        setVoltage();
      } else {
        reachSetPoint(getSetpoint());
      }
      if (masterMotor.getEncoder().getVelocity() < 0.5 && masterMotor.getEncoder().getVelocity() > -0.5) {
        masterMotor.getEncoder().setPosition(getCurrentAngle() * (Constants.ArmConstants.GEARING / 360.0));
      }
      pidController.setP(kP.lastValue);
      pidController.setI(kI.lastValue);
      pidController.setD(kD.lastValue);
      SmartDashboard.putNumber("Arm/CurrentAngle", getCurrentAngle());
      SmartDashboard.putNumber("Arm/SetPoint", getSetpoint());
      SmartDashboard.putNumber("Arm/MotorVelocity", masterMotor.getEncoder().getVelocity());
      SmartDashboard.putNumber("Arm/MotorOutput", masterMotor.getAppliedOutput());
      SmartDashboard.putNumber("Arm/MotorCurrent", masterMotor.getOutputCurrent());
      // This method will be called once per scheduler run
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void configureMotors() {
    masterConfig.idleMode(IdleMode.kBrake).voltageCompensation(12.0).smartCurrentLimit(45).encoder
        .positionConversionFactor(Constants.ArmConstants.ENCODER_DISTANCE_PER_PULSE * Constants.ArmConstants.GEARING);
    slaveConfig.idleMode(IdleMode.kBrake).follow(masterMotor).voltageCompensation(12.0).smartCurrentLimit(45);

    masterMotor.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    slaveMotor.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void configureEncoder() {
    quadEncoder.reset();
    quadEncoder.setDistancePerPulse(0.6);
  }
}
