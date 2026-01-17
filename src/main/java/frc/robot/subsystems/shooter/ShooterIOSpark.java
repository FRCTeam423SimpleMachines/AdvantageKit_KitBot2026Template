package frc.robot.subsystems.shooter;

import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

<<<<<<< Updated upstream
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import static frc.robot.util.SparkUtil.ifOk;

=======
>>>>>>> Stashed changes
public class ShooterIOSpark implements ShooterIO {
  private final SparkFlex shooter =
      new SparkFlex(ShooterConstants.shooterCanID, MotorType.kBrushless);
  private final RelativeEncoder shooterEncoder = shooter.getEncoder();
  private final PIDController pid = 
      new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
  private final SimpleMotorFeedforward feedforward = 
      new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV);
  private double TargetRPM;


  public ShooterIOSpark() {}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    ifOk(shooter, shooterEncoder::getVelocity, (value) -> inputs.flywheelRPM = value);
  }

  @Override
<<<<<<< Updated upstream
  public void runShooter(double voltage) {
    shooter.setVoltage(voltage);
=======
  public void runShooter(double speed) {
    // TODO Auto-generated method stub
    shooter.set(speed);
>>>>>>> Stashed changes
  }

  @Override
  public void runAtSpeed(double speed) {
    shooter.set(speed);
  }

  @Override
  public void runAtTarget() {
    shooter.setVoltage(pid.calculate(shooterEncoder.getVelocity(), TargetRPM) + feedforward.calculate(TargetRPM));
  }

  @Override
  public void runAtTarget(double RPM) {
    shooter.setVoltage(pid.calculate(shooterEncoder.getVelocity(), RPM) + feedforward.calculate(RPM));
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
    shooter.setVoltage(pid.calculate(shooterEncoder.getVelocity(), TargetRPM) + feedforward.calculate(TargetRPM));
  }
}
