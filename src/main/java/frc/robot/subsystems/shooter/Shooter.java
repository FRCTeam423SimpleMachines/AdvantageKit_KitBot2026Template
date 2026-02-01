package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void runShooter(double speed) {
    io.runShooter(speed);
  }

  public void incrementTargetRPM(double increment) {
    io.incrementTargetRPM(increment);
  }

  public void setTargetRPM(double RPM) {
    io.setTargetRPM(RPM);
  }

  public void runSecondFlywheel(double speed) {
    io.setSecondFlywheel(speed);
  }

  public Command runAtTarget() {
    return run(() -> io.runAtTarget());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }
}
