package frc.robot.commands.climber;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberPosition;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.Climber;

public class ClimberJoystickCommand extends CommandBase {

  // private static final double CLIBER_UPPER_SOFT_LIMIT = 3.0;
  // private static final double CLIBER_LOWER_SOFT_LIMIT = 0.4;
  private static final double MAX_SPEED = 0.10;

  private final ClimberPosition position;
  private final Supplier<Boolean> isOverridingLimits;
  private final Climber climber;
  private final XboxController controller;

  // TODO: Internal?
  private PIDController climberController = new PIDController(2, 0.0, 0.0);

  private boolean shouldHoldPositionWhenStopped = true;

  /*
  public ClimberJoystickCommand(XboxController controller, Climber climber) {
    this.climber = climber;
    this.controller = controller;

    addRequirements(climber);
  }
  */

  public ClimberJoystickCommand(ClimberPosition position, Supplier<Boolean> isOverridingLimits, XboxController controller, Climber climber) {
    this.position = position;
    this.climber = climber;
    this.controller = controller;

    addRequirements(climber);
  }

  @Override
  public void initialize() {
    shouldHoldPositionWhenStopped = true;

    climberController.reset();
  }

  @Override
  public void execute() {
    // TODO: Needs to be override switch to rumble
    if (climber.isStringPotConnected()) {
      startWarningRumble();
    } else {
      stopWarningRumble();
    }

    double joystick = -Utilities.deadband(controller.getRightY(), 0.15);
    if (joystick == 0) {
      // If our joystick is not being moved, hold our climber in it's current position
      if (shouldHoldPositionWhenStopped) {
        climber.holdAtCurrentPosition();
        shouldHoldPositionWhenStopped = false;
      }
    } else {
      double output = 0.0;

      // TODO: Scale this based on a speed!
      output = joystick;

      // if (climber.isStringPotConnected()) {
      //   final double stringPotVoltage = climber.getStringPotVoltage();
      //   output = climberController.calculate(stringPotVoltage, setpoint);
      //   // TODO: These ranges are bad because it will allow us to overdrive slight
      //   // Either fix it in software, or fix it in hardware.
      //   // Never drive our climber BELOW it's starting position
      //   if (output < 0 && stringPotVoltage <= climber.getStartStringPotVoltage()) {
      //     output = 0.0;
      //   }
      //   // Never drive our climber ABOVE our setpoint
      //   if (output > 0 && stringPotVoltage >= setpoint) {
      //     output = 0.0;
      //   }
      // } else {
      //   // Use String Pot unless it below the soft limit of the physical climber (broken or disconnected)
      //   // Be careful driving the climber with the string pot disconnected!
      //   // TODO: We also need the override thing in here...
      //   startWarningRumble();
      //   output = joystick;
      //   // TODO: Watch the internal
      // }

      // TODO: These ranges are bad because it will allow us to overdrive slight
      // Final check to makes sure we never drive our climber above/below the soft limits set

      output = MathUtil.clamp(
        joystick,
        -MAX_SPEED,
        MAX_SPEED
      );
      climber.setSpeed(output);

      shouldHoldPositionWhenStopped = true;
    }
  }

  private void startWarningRumble() {
    controller.setRumble(RumbleType.kRightRumble, 0.1);
  }

  private void stopWarningRumble() {
    controller.setRumble(RumbleType.kRightRumble, 0.0);
  }

  @Override
  public void end(boolean interupted) {
    stopWarningRumble();
    climber.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
