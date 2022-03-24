package frc.robot.commands.auto;

import org.ejml.equation.IntegerSequence.For;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.delivery.AutoStartDelivery;
import frc.robot.commands.kicker.ForwardKickerCommand;
import frc.robot.commands.kicker.StartKicker;
import frc.robot.commands.shooter.StartShooterInstantCommand;
import frc.robot.commands.swerve.MaintainHeadingCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class Test extends SequentialCommandGroup {

  public Test(AutoDrive autoDrive, Delivery delivery, Drivetrain drivetrain, Heading heading, Kicker kicker, Shooter shooter) {
    addCommands(
      new ProfiledPointToPointWithHeading(Constants.Auto.kPosition3RightStart, drivetrain::getTranslation, 15, 1.0, 0.05, Units.inchesToMeters(120), 8, autoDrive, heading)
    );
  }

}