package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double flywheelRPM = 0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void runShooter(double voltage) {}
}
