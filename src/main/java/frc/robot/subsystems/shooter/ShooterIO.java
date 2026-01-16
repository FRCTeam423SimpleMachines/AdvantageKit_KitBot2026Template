package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double flywheelRPM = 0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void runShooter(double voltage) {}

  public default void runAtSpeed(double speed) {}

  public default void runAtTarget() {}

  public default void runAtTarget(double RPM) {}

  public default void setTargetRPM(double RPM) {}

  public default void incrementTargetRPM(double increment) {}

  public default void setTargetRun(double RPM) {}
  }

