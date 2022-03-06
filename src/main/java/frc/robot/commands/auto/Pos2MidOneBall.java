package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class Pos2MidOneBall extends SequentialCommandGroup {

  public Pos2MidOneBall(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
    addCommands(
      new ProfiledPointToPointCommand(Constants.Auto.kBallR2, drivetrain::getTranslation, 3.0, 0.05, Units.inchesToMeters(120), 8, autoDrive, heading),
      new WaitCommand(1)
    );
  }

}