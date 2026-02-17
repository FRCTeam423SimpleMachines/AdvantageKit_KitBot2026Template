package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    // Encoder position (rotations or converted units)
    public double position = 0.0;

    // Encoder velocity (rotations per minute or converted units)
    public double velocity = 0.0;

    // Applied voltage
    public double appliedVolts = 0.0;

    // Output current
    public double currentAmps = 0.0;

    // Target speed (for logging)
    public double targetSpeed = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  // Open-loop control by voltage (volts)
  public default void setVoltage(double volts) {}

  // Open-loop control by percent output (-1..1)
  public default void setSpeed(double speed) {}
}
