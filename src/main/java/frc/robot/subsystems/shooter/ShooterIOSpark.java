package frc.robot.subsystems.shooter;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.function.DoubleSupplier;

public class ShooterIOSpark implements ShooterIO {
  private final SparkFlex shooter =
      new SparkFlex(ShooterConstants.shooterCanID, MotorType.kBrushless);
  private final RelativeEncoder shooterEncoder = shooter.getEncoder();
  private final SparkClosedLoopController shooterController = shooter.getClosedLoopController();
  private final SparkFlex shooter2 =
      new SparkFlex(ShooterConstants.secondShooterCanID, MotorType.kBrushless);
  private final RelativeEncoder shooter2Encoder = shooter2.getEncoder();
  private final DigitalInput laser1 = new DigitalInput(0);
  private final DigitalInput laser2 = new DigitalInput(1);
  private double TargetRPM = 0;

  // no persistent config fields needed; configs are built during construction

  public ShooterIOSpark() {
    // Build base config for shooter motors and a follower config for the second motor
    var base = new SparkFlexConfig();
    base.idleMode(IdleMode.kCoast).voltageCompensation(12.0);
    base.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    base.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD)
        .feedForward
        .kV(ShooterConstants.kV)
        .kA(ShooterConstants.kA);

    // Create a follower config based on base that follows the leader's CAN ID
    SparkFlexConfig follower = new SparkFlexConfig();
    follower.follow(ShooterConstants.shooterCanID, true);

    // Try configuring the controllers until successful (handles transient CAN errors)
    // Apply base config to both controllers first so they both have the same base settings,
    // then apply the follower config to the second controller to enable following mode.
    tryUntilOk(
        shooter,
        5,
        () ->
            shooter.configure(
                base, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        shooter2,
        5,
        () ->
            shooter2.configure(
                base, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        shooter2,
        5,
        () ->
            shooter2.configure(
                follower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Reset sticky fault and read sensors only if the controller reports OK
    sparkStickyFault = false;
    ifOk(shooter, shooterEncoder::getVelocity, (value) -> inputs.flywheelRPM = value);
    ifOk(shooter2, shooter2Encoder::getVelocity, (value) -> inputs.secondFlywheelRPM = value);
    ifOk(
        shooter,
        new DoubleSupplier[] {shooter::getAppliedOutput, shooter::getBusVoltage},
        (values) -> {});
    ifOk(shooter, shooter::getOutputCurrent, (value) -> {});
    inputs.targetRPM = TargetRPM;
    inputs.laser1 = laser1.get();
    inputs.laser2 = laser2.get();
  }

  @Override
  public void runShooter(double voltage) {
    shooter.setVoltage(voltage);
  }

  @Override
  public void runAtSpeed(double speed) {
    shooter.set(speed);
  }

  @Override
  public void runAtTarget() {
    // Feedforward is configured in the motor closed-loop config; use the controller's setpoint
    shooterController.setSetpoint(TargetRPM, ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void runAtTarget(double RPM) {
    shooterController.setSetpoint(RPM, ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void setTargetRPM(double RPM) {
    TargetRPM = RPM;
  }

  @Override
  public void incrementTargetRPM(double increment) {
    TargetRPM = TargetRPM + increment;
  }

  @Override
  public void setTargetRun(double RPM) {
    TargetRPM = RPM;
    shooterController.setSetpoint(RPM, ControlType.kMAXMotionVelocityControl);
  }
}
