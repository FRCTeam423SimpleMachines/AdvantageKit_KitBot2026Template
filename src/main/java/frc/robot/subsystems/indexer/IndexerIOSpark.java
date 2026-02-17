package frc.robot.subsystems.indexer;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Spark-based implementation of the Indexer IO. Uses two Spark Max controllers. */
public class IndexerIOSpark implements IndexerIO {
  private final SparkMax indexer =
      new SparkMax(IndexerConstants.indexerCanId, MotorType.kBrushless);
  private final SparkMax indexer2 =
      new SparkMax(IndexerConstants.indexer2CanId, MotorType.kBrushless);

  private final RelativeEncoder indexerEncoder = indexer.getEncoder();

  public IndexerIOSpark() {
    // Configure base config and make second motor follow the first via config.follow
    var base = new SparkMaxConfig();
    base.idleMode(IdleMode.kBrake).voltageCompensation(12.0);

    // Create a follower config based on base that follows the leader's CAN ID
    SparkBaseConfig followerConfig = new SparkMaxConfig();
    followerConfig.follow(IndexerConstants.indexerCanId, true);

    tryUntilOk(
        indexer,
        5,
        () ->
            indexer.configure(
                base, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        indexer,
        5,
        () ->
            indexer2.configure(
                base, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        indexer2,
        5,
        () ->
            indexer2.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    ifOk(indexer, indexerEncoder::getPosition, (value) -> inputs.position = value);
    ifOk(indexer, indexerEncoder::getVelocity, (value) -> inputs.velocity = value);
    ifOk(indexer, indexer::getOutputCurrent, (value) -> inputs.currentAmps = value);
    ifOk(
        indexer,
        new java.util.function.DoubleSupplier[] {indexer::getAppliedOutput, indexer::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
  }

  @Override
  public void setVoltage(double volts) {
    indexer.setVoltage(volts);
    indexer2.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    indexer.set(speed);
  }
}
