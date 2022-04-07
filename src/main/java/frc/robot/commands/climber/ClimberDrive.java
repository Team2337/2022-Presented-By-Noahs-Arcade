package frc.robot.commands.climber;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.oi.NerdyOperatorStation;
import frc.robot.subsystems.Climber;

/**
 * This command runs the climber to a setpoint using FalconFX encoders.
 * @author Nicholas S
 */
public class ClimberDrive extends CommandBase {
  private final Climber climber;
  private double speed;
  private boolean shouldHoldPositionWhenStopped = true;
  private final NerdyOperatorStation operatorStation;
  private final Supplier<Rotation2d> rollSupplier;

  /**
   * 
   * @param setpoint
   * @param climber
   */
  public ClimberDrive(double speed, Supplier<Rotation2d> rollSupplier, NerdyOperatorStation operatorStation, Climber climber) {
    this.climber = climber;
    this.speed = speed;
    this.rollSupplier = rollSupplier;
    this.operatorStation = operatorStation;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    shouldHoldPositionWhenStopped = true;
  }

  @Override
  public void execute() {
    // Deadband makes sure slight inaccuracies in the controller does not make the
    // controller move if it isn't touched
    if (speed == 0) {
      // If our joystick is not being moved, hold our climber in it's current position
      if (shouldHoldPositionWhenStopped) {
        climber.hold();
        shouldHoldPositionWhenStopped = false;
      }
    } else {
      if (climber.getStringPotVoltage() > 2.8 && !operatorStation.blackSwitch.get()) {
        climber.releaseServos();
      }
      if (((speed > 0) && (rollSupplier.get().getDegrees() > Constants.CLIMBER_ROLL) && ((climber.getStringPotVoltage() < 2.0) && (climber.getStringPotVoltage() > 1.59))) && operatorStation.blueSwitch.get()) {
        speed = 0;
      }
      if (operatorStation.yellowSwitch.get()) {
        climber.setSpeed(speed);
      } else {
        climber.setSpeed(0);
      }
      shouldHoldPositionWhenStopped = true;
    }
  }

  @Override
  public void end(boolean interupted) {
    climber.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}