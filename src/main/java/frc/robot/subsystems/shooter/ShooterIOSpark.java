package frc.robot.subsystems.shooter;

import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterIOSpark implements ShooterIO {
  private final SparkFlex shooter =
      new SparkFlex(ShooterConstants.shooterCanID, MotorType.kBrushless);
  private final SparkFlex shooter2 =
      new SparkFlex(ShooterConstants.secondShooterCanID, MotorType.kBrushless);
  private final SparkMax turret = 
      new SparkMax(ShooterConstants.turretCanID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder turretEncoder = turret.getEncoder(); 
  private final RelativeEncoder shooterEncoder = shooter.getEncoder();
  private final PIDController pid =
      new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV);
  private final PIDController turretPID = 
      new PIDController(ShooterConstants.turretkP, ShooterConstants.turretkI, ShooterConstants.turretkD);
  //private final SparkFlex shooter2 =
   //   new SparkFlex(ShooterConstants.secondShooterCanID, MotorType.kBrushless);
  private final DigitalInput laser1 = new DigitalInput(0);
  private final DigitalInput laser2 = new DigitalInput(1);
  private double TargetRPM = 0;

  public ShooterIOSpark() {}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    ifOk(shooter, shooterEncoder::getVelocity, (value) -> inputs.flywheelRPM = value);
    inputs.targetRPM = TargetRPM;
    pid.setTolerance(500);
    turretPID.setTolerance(3);
    inputs.laser1 = laser1.get();
    inputs.laser2 = laser2.get();
  }

  @Override
  public void runShooter(double voltage) {
    shooter.setVoltage(voltage);
  }

  @Override
  public void runAtSpeed(double speed) {
    shooter.set(speed);
  }

  @Override
  public void runAtTarget() {
    double output =
        ((pid.calculate(shooterEncoder.getVelocity(), TargetRPM) + feedforward.calculate(TargetRPM))
            / 6000.0);
    shooter.set(output);
    shooter2.set(-output);
  }

  @Override
  public void runAtTarget(double RPM) {
    shooter.set(
        (pid.calculate(shooterEncoder.getVelocity(), RPM) + feedforward.calculate(RPM)) / 6000.0);
  }

  @Override
  public void setTargetRPM(double RPM) {
    TargetRPM = RPM;
  }

  @Override
  public void incrementTargetRPM(double increment) {
    TargetRPM = TargetRPM + increment;
  }

  @Override
  public void setTargetRun(double RPM) {
    TargetRPM = RPM;
    shooter.set((
        pid.calculate(shooterEncoder.getVelocity(), TargetRPM) + feedforward.calculate(TargetRPM)) / 6000.0);
  }

  @Override
  public void setSecondFlywheel(double speed) {
    shooter2.set(speed);
  }

  @Override
  public void setTurretSpeed(double speed) {
    turret.set(speed);
  }

  @Override
  public void setTurretAngle(double angle) {
    double output = 
      (turretPID.calculate(turretEncoder.getPosition(), angle));
    turret.set(output);
  }
}
