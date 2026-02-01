package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double flywheelRPM = 0;
    public double targetRPM = 0;
    public double secondFlywheelRPM = 0;
    public boolean laser1 = false;
    public boolean laser2 = false;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void runAtSpeed(double speed) {}

  public default void runAtTarget() {}

  public default void runAtTarget(double RPM) {}

  public default void setTargetRPM(double RPM) {}

  public default void incrementTargetRPM(double increment) {}

  public default void setTargetRun(double RPM) {}

  public default void runShooter(double speed) {}

  public default void setSecondFlywheel(double speed) {}
}
