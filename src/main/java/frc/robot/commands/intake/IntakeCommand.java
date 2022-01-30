package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Delivery;
import frc.robot.Constants.Directions;
import frc.robot.commands.delivery.*;

/**
 * Determines if we need to start the intake.
 * 
 * @author Nicholas S
 */
public class IntakeCommand extends CommandBase {
  // The subsystem the command runs on
  private final Intake subsystem;
  private final Delivery delivery;

  public IntakeCommand(Intake m_subsystem, Delivery delivery) {
    subsystem = m_subsystem;
    this.delivery = delivery;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    // If there are two balls in the delivery, we cannot have any balls, so reverse the intake
    if (delivery.balls == 2) {
      subsystem.reverseIntake();
    } else {
      // If there is room, start intaking a ball in
      subsystem.startFirstStage();
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Start the feeding of the ball into the delivery
    subsystem.startSecondStage();

    // If the system will be full, reverse the first stage to stop any other balls from entering
    if (delivery.balls == 1) {
      subsystem.reverseFirstStage();
    }
    if(delivery.storedBalls[0] != null){
      new CheckCommand(delivery, Directions.COUNTER_CLOCKWISE);
    }
    // Wait half a second to makes sure ball reaches the delivery, and then process the ball. 
    new WaitCommand(500).andThen(new AfterIntakeCommand(delivery));
  }

  @Override
  public boolean isFinished() {
    //When a ball breaks the first stage of the intake
    return delivery.getIntakeSensorStatus();
  }
}
