package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Delivery;

/**
 * Determines if we need to start the intake.
 * 
 * @author Nicholas S
 */
public class IntakeCommand extends CommandBase {
  // The subsystem the command runs on
  private final Intake intake;
  private final Delivery delivery;

  public IntakeCommand(Intake intake, Delivery delivery) {
    this.intake = intake;
    this.delivery = delivery;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    // If there are two balls in the delivery, we cannot have any balls, so reverse the intake
    if (delivery.getNumberOfBalls() == 2) {
      intake.reverseIntake();
    } else {
      // If there is room, start intaking a ball in
      intake.startIntake();
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Start the feeding of the ball into the delivery
    intake.stopIntake();

    // If the system will be full, reverse the first stage to stop any other balls from entering
    if (delivery.getNumberOfBalls() == 1) {
      intake.reverseIntake();
    }
  }

  @Override
  public boolean isFinished() {
    //When a ball breaks the first stage of the intake
    return intake.getIntakeSensorStatus();
  }
}
