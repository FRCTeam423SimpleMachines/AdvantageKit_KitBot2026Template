package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }
}
