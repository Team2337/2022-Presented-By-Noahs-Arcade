package frc.robot.commands.teleop;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.commands.HeadingToTargetCommand;
import frc.robot.commands.interfaces.AutoDrivableCommand;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Heading;

/**
 * Drive a certain distance from a point - regardless of angle - and maintain that
 * distance away from the point. Will attempt to face target while driving towards
 * target.
 */
public class DistanceToTargetCommand extends HeadingToTargetCommand implements AutoDrivableCommand {

  private double distanceMeters;
  private AutoDrive autoDrive;

  private ProfiledPIDController distanceController;
  private double output = 0.0;

  public DistanceToTargetCommand(double distanceMeters, Supplier<Translation2d> translationSupplier, AutoDrive autoDrive, Heading heading) {
    this(Constants.kHub, distanceMeters, translationSupplier, autoDrive, heading);
  }

  public DistanceToTargetCommand(Translation2d targetMeters, double distanceMeters, Supplier<Translation2d> translationSupplier, AutoDrive autoDrive, Heading heading) {
    super(targetMeters, translationSupplier, heading);

    this.distanceMeters = distanceMeters;
    this.autoDrive = autoDrive;

    distanceController = new ProfiledPIDController(1.5, 0.0, 0.0,
      // Speeds have been tuned for 143" move (Ball 2 -> 10in from Hub)
      new TrapezoidProfile.Constraints(
        Units.inchesToMeters(160),
        Units.inchesToMeters(80)
      )
    );
    // Tolerance for our distance is within ~2 inches (should be Good Enough™️)
    distanceController.setTolerance(Units.inchesToMeters(2));

    addRequirements(autoDrive);
  }

  @Override
  public void initialize() {
    super.initialize();

    autoDrive.setDelegate(this);

    // Set our initial setpoint for our profiled PID controllers
    // to avoid a JUMP to their starting values on first run
    PolarCoordinate robotCoordinate = getRobotCoordinate();
    distanceController.reset(
      robotCoordinate.getRadiusMeters()
    );
  }

  @Override
  public void execute() {
    super.execute();

    PolarCoordinate robotCoordinate = getRobotCoordinate();
    output = distanceController.calculate(
      robotCoordinate.getRadiusMeters(),
      distanceMeters
    );

    Logger.getInstance().recordOutput("Flare/output", output);
    Logger.getInstance().recordOutput("Flare/Error", distanceController.getPositionError());
    Logger.getInstance().recordOutput("Flare/Velocity Error", distanceController.getVelocityError());
    Logger.getInstance().recordOutput("Flare/atTarget", distanceController.atGoal());

    // Clamp to some max speed (should be between [0.0, 1.0])
    final double maxSpeed = 1.0;
    output = MathUtil.clamp(
      output,
      -maxSpeed,
      maxSpeed
    );
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    autoDrive.clearDelegate();
  }

  public AutoDrive.State calculate(double forward, double strafe, boolean isFieldOriented) {
    // Use our distanController output calculation as our forward value.
    // Driver can strafe during this command. Forward should be forward to the robot.
    // Note that this command assumes we're facing the target (use with Heading)
    return new AutoDrive.State(
      -output,
      strafe,
      false
    );
  }

}