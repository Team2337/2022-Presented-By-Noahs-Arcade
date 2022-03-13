package frc.robot.commands.climber;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.delivery.AutoStopDelivery;
import frc.robot.commands.intake.AutoStopIntake;
import frc.robot.commands.shooter.StopShooterInstantCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class ClimbSequenceCommandGroup extends SequentialCommandGroup {

  public ClimbSequenceCommandGroup(Supplier<Rotation2d> robotPitch, XboxController controller, Climber climber) {
    addCommands(
      new MoveToThirdRung(controller, climber),
      //Shoot out poles (Requires Servos)
      new MoveToThirdRung(controller, climber),
      new MovePastThirdRung(robotPitch, controller, climber),
      new MoveToHooksSet(controller, climber),
      new LiftUp(controller, climber)
    );

  }
}
