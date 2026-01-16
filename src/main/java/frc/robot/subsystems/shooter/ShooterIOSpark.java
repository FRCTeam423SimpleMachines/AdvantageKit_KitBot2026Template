package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.util.SparkUtil.ifOk;

public class ShooterIOSpark implements ShooterIO {
  private final SparkFlex shooter =
      new SparkFlex(ShooterConstants.shooterCanID, MotorType.kBrushless);
  private final RelativeEncoder shooterEncoder = shooter.getEncoder();

  public ShooterIOSpark() {}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    ifOk(shooter, shooterEncoder::getVelocity, (value) -> inputs.flywheelRPM = value);
  }

  @Override
  public void runShooter(double voltage) {
    // TODO Auto-generated method stub
    shooter.setVoltage(voltage);
  }
}
