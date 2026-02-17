package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  private final DCMotorSim m_indexer;

  public IndexerIOSim() {
    m_indexer =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.004, 10),
            DCMotor.getNeo550(1));
  }

  // Open-loop control by voltage (volts)
  public void setVoltage(double volts) {
    m_indexer.setInputVoltage(volts);
  }

  // Open-loop control by percent output (-1..1)
  public void setSpeed(double speed) {
    m_indexer.setInput(speed);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {}
}
