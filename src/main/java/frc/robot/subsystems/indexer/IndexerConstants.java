package frc.robot.subsystems.indexer;

/** Constants for the Indexer subsystem. Update CAN IDs and tuning constants for your robot. */
public final class IndexerConstants {
  private IndexerConstants() {}

  // CAN IDs for the two Spark Max controllers used by the indexer
  public static final int indexerCanId = 13;
  public static final int indexer2CanId = 14;

  // Basic tuning placeholders (fill in if you run closed-loop control on the leader)
  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
}
