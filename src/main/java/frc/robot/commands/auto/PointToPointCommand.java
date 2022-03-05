package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.HeadingToTargetCommand;
import frc.robot.commands.interfaces.AutoDrivableCommand;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Heading;

public class PointToPointCommand extends HeadingToTargetCommand implements AutoDrivableCommand {

  private static final double kDistanceP = 0.004;
  private static final double kThetaP = 0.004;

  private PolarCoordinate target;
  private Heading heading;
  private AutoDrive autoDrive;

  private double forwardOutput = 0.0;
  private double strafeOutput = 0.0;

  public PointToPointCommand(PolarCoordinate target, Supplier<Translation2d> translationSupplier, AutoDrive autoDrive, Heading heading) {
    super(
      target.getReferencePoint(),
      translationSupplier,
      heading
    );

    this.target = target;
    this.heading = heading;
    this.autoDrive = autoDrive;

    addRequirements(autoDrive);

    log(getRobotCoordinate());
  }

  @Override
  public void initialize() {
    super.initialize();

    heading.enableMaintainHeading();
    autoDrive.setDelegate(this);
  }

  @Override
  public void execute() {
    super.execute();

    PolarCoordinate robotCoordinate = getRobotCoordinate();
    /*
    forwardOutput = distanceController.calculate(
      robotCoordinate.getRadiusMeters(),
      target.getRadiusMeters()
    );
    strafeOutput = thetaController.calculate(
      robotCoordinate.getTheta().getDegrees(),
      Utilities.convertRotationToRelativeRotation(target.getTheta()).getDegrees()
    );
    */

    // Clamp to some max speed (should be between [0.0, 1.0])
    final double maxSpeed = 1.0;
    forwardOutput = MathUtil.clamp(
      forwardOutput,
      -maxSpeed,
      maxSpeed
    );

    strafeOutput = MathUtil.clamp(
      strafeOutput,
      -maxSpeed,
      maxSpeed
    );

    log(robotCoordinate);
  }

  public AutoDrive.State calculate(double forward, double strafe, boolean isFieldOriented) {
    return new AutoDrive.State(
      -forwardOutput,
      -strafeOutput
    );
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    autoDrive.clearDelegate();
  }

  @Override
  public boolean isFinished() {
    return distanceController.atGoal() && thetaController.atGoal();
  }

  private void log(PolarCoordinate robotCoordinate) {
    SmartDashboard.putNumber("P2P/Target Distance (inches)", Units.metersToInches(target.getRadiusMeters()));
    SmartDashboard.putNumber("P2P/Target Theta (Degrees)", target.getTheta().getDegrees());

    SmartDashboard.putNumber("P2P/Robot Angle (degrees)", robotCoordinate.getTheta().getDegrees());
    SmartDashboard.putNumber("P2P/Robot Distance (inches)", Units.metersToInches(robotCoordinate.getRadiusMeters()));

    SmartDashboard.putNumber("P2P/Forward Output", forwardOutput);
    SmartDashboard.putNumber("P2P/Strafe Output", strafeOutput);

    // SmartDashboard.putNumber("P2P/Distance Position (inches)", Units.metersToInches(distanceController.getSetpoint().position));
    // SmartDashboard.putNumber("P2P/Distance Error (inches)", Units.metersToInches(distanceController.getPositionError()));

    // SmartDashboard.putNumber("P2P/Theta Position", thetaController.getSetpoint().position);
    // SmartDashboard.putNumber("P2P/Theta Error", thetaController.getPositionError());

    // SmartDashboard.putBoolean("P2P/distanceController atGoal", distanceController.atGoal());
    // SmartDashboard.putBoolean("P2P/thetaController atGoal", thetaController.atGoal());
  }

}
