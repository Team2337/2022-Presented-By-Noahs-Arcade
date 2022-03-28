package frc.robot.commands.climber;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.oi.NerdyOperatorStation;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.Climber;

public class ClimberJoystickCommand extends CommandBase {

  private final Climber climber;
  private final XboxController controller;
  private final NerdyOperatorStation operatorStation;
  private final Supplier<Rotation2d> rollSupplier;

  private boolean shouldHoldPositionWhenStopped = true;

  public ClimberJoystickCommand(Supplier<Rotation2d> rollSupplier, XboxController controller, NerdyOperatorStation operatorStation, Climber climber) {
    this.climber = climber;
    this.controller = controller;
    this.operatorStation = operatorStation;
    this.rollSupplier = rollSupplier;

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
    double output = Utilities.deadband(controller.getRightY(), 0.15);
    if (output == 0) {
      // If our joystick is not being moved, hold our climber in it's current position
      if (shouldHoldPositionWhenStopped) {
        climber.hold();
        shouldHoldPositionWhenStopped = false;
      }
    } else {
      if (climber.getStringPotVoltage() > 2.8 && !operatorStation.blackSwitch.get()) {
        climber.releaseServos();
      }
      if (((output > 0) && (rollSupplier.get().getDegrees() > Constants.CLIMBER_ROLL) && ((climber.getStringPotVoltage() < 2.0) && (climber.getStringPotVoltage() > 1.59))) && operatorStation.blueSwitch.get()) {
        output = 0;
      } 
      climber.setSpeed(output);
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
