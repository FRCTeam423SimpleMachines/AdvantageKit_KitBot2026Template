package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  private final DCMotorSim m_flywheel;

  public ShooterIOSim() {
    m_flywheel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.004, 10),
            DCMotor.getNeo550(1));
  }

  @Override
  public void runShooter(double volts) {
    m_flywheel.setInputVoltage(volts);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {}
}
