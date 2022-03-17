package frc.robot.commands.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.Climber;

public class ClimberJoystickCommand extends CommandBase {

  private static final double MAX_SPEED = 0.7;

  private final Climber climber;
  private final XboxController controller;

  private boolean shouldHoldPositionWhenStopped = true;
  private double MIN_STRINGPOT_VALUE = 0.4;
  private double MAX_STRINGPOT_VALUE = 3.00;

  public ClimberJoystickCommand(XboxController controller, Climber climber) {
    this.climber = climber;
    this.controller = controller;

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
    double joystick = -Utilities.deadband(controller.getRightY(), 0.15);
    if (joystick == 0) {
      // If our joystick is not being moved, hold our climber in it's current position
      if (shouldHoldPositionWhenStopped) {
        climber.hold();
        shouldHoldPositionWhenStopped = false;
      }
    } else {
      double output = MathUtil.clamp(
        joystick,
        -MAX_SPEED,
        MAX_SPEED
      );
      if (((climber.getStringPotVoltage() < MIN_STRINGPOT_VALUE) && (output > 0.0)) || ((climber.getStringPotVoltage() > MAX_STRINGPOT_VALUE) && (output < 0))) {
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
