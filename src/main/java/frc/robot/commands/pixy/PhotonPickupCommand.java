package frc.robot.commands.pixy;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Vision;
import frc.robot.commands.interfaces.AutoDrivableCommand;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.AutoDrive.State;
import frc.robot.subsystems.hardware.PhotonVision;
import frc.robot.subsystems.AutoDrive;

public class PhotonPickupCommand extends CommandBase implements AutoDrivableCommand {

  /**
   * Whatever ball color we want to pick up. Red, blue, or any.
   */
  public static enum PickupStrategy {
    RED,
    BLUE,
    ANY,
    OURS,
    THEIRS
  }

  // The position in the frame where the ball will go max speed.
  // Ex: 1 == at the far edge of the frame, the ball will go max speed.
  // If the ball is further in the frame, we will scale our speed.
  // Ex: 2 == until the ball is halfway through the half frame (so 1/4
  // through the full frame) we will go full speed towards it, then
  // start scaling our speed.
  // This is useful for the buffer zones on the side of the frame where
  // we don't detect a ball right away but don't want to go slower until
  // we absolutely have to.
  private static final double MAX_SPEED_HALF_FRAME_SCALE = 1.1;
  private static final double MAX_STRAFE_OUTPUT = 0.5;
  private static final double LAST_SEEN_CYCLE_COUNTER_MAX = 50; // 1s

  private final PickupStrategy strategy;
  private final AutoDrive autoDrive;

  private final PIDController strafeController = new PIDController(0.0035, 0.0, 0.0); //P 0.0035

  // private var targetBall;
  private Double targetX = null;
  private int lastSeenCycleCounter = 0;
  private double strafeOutput = 0.0;
  private XboxController driverController;

  public Supplier<Rotation2d> gyroSupplier;

  public double spangle = 0;

  private PolarCoordinate joystickCoordinates = new PolarCoordinate(0, Rotation2d.fromDegrees(0));

  private PhotonCamera camera = new PhotonCamera("intake");
  private PhotonPipelineResult targets;
  private PhotonTrackedTarget ball;
  private double yawValue = 0.0;
  private boolean hasTarget = false;
  private List<TargetCorner> corners;
  private Double ballX;

  public PhotonPickupCommand(PickupStrategy strategy, Supplier<Rotation2d> gyroSupplier, XboxController driverController, AutoDrive autoDrive) {
    this.strategy = strategy;
    this.gyroSupplier = gyroSupplier;
    this.driverController = driverController;
    this.autoDrive = autoDrive;

    addRequirements(autoDrive);
  }

  @Override
  public void initialize() {
    autoDrive.setDelegate(this);
    resetInternalState();
  }

  private void log() {
    String strategyString = "N/A";
    if (strategy != null) {
      strategyString = strategy.toString();
    }
    SmartDashboard.putString("Target X", String.valueOf(targetX));
    SmartDashboard.putString("PixyPickup/Strategy", strategyString);
    SmartDashboard.putNumber("PixyPickup/Last Seen Counter", lastSeenCycleCounter);
    SmartDashboard.putNumber("PixyPickup/Strafe Output", strafeOutput);
    SmartDashboard.putNumber("PixyPickup/Controller Error", strafeController.getPositionError());
  }

  private void resetInternalState() {
    targetX = null;
    lastSeenCycleCounter = 0;
    strafeOutput = 0.0;

    strafeController.reset();
  }

  @Override
  public void execute() {
    log();
    targets = camera.getLatestResult();
    if (targets.hasTargets()) {
      ball = targets.getBestTarget();
      yawValue = ball.getYaw();
      corners = ball.getCorners();
      hasTarget = true;
      ballX = getXValue(corners);
      SmartDashboard.putNumber("Photon X", ballX);
    } else {
      hasTarget = false;
      ballX = null;
    }


    double y = driverController.getLeftX();
    double x = -driverController.getLeftY();
    Rotation2d joystickAngle = joystickCoordinates.fromFieldCoordinate(new Translation2d(x, y), new Translation2d(0, 0)).getTheta();
    double jAngle = -joystickAngle.getDegrees();
    double gAngle = gyroSupplier.get().getDegrees();
    double dangle = jAngle - gAngle;

    spangle = Math.cos(Math.toRadians(dangle));
    spangle = spangle * 0.4;
    
    if (Math.abs(x) < 0.1 && Math.abs(y) < 0.1) {
      spangle = 0;
    }

    if (strategy == null) {
      return;
    }
    //Add the pipelines to 
    if (strategy == PickupStrategy.OURS) {
      switch (DriverStation.getAlliance()) {
        default:
        case Red:
          changePipeline(Vision.RED_PIPELINE_INDEX);
          break;
        case Blue:
          changePipeline(Vision.BLUE_PIPELINE_INDEX);
          break;
        } 
      } else if (strategy == PickupStrategy.THEIRS) {
        switch (DriverStation.getAlliance()) {
          default:
          case Red:
            changePipeline(Vision.BLUE_PIPELINE_INDEX);
            break;
          case Blue:
            changePipeline(Vision.RED_PIPELINE_INDEX);
            break;
          } 
      }

    Double latestTargetX = getBallXValue();
    // If our `latestTargetBall` is null, remember where our last seen ball was
    // until we haven't seen a new ball ball for LAST_SEEN_CYCLE_COUNTER_MAX
    // number of cycles.
    if (targetX != null && latestTargetX == null) {
      lastSeenCycleCounter++;
    } else {
      lastSeenCycleCounter = 0;
      targetX = latestTargetX;
    }
    if (lastSeenCycleCounter >= LAST_SEEN_CYCLE_COUNTER_MAX) {
      targetX = null;
    }

    strafeOutput = 0.0;

    if (targetX == null) {
      return;
    }

    // The first time we see a ball, we should turn the intake on
    // intake.start();

    double frameCenter = getFrameCenter();
    double output = strafeController.calculate(
      (double) targetX,
      frameCenter
    );

    // Determine our maximum output based on the half-frame size + our P value
    // and scale our output so we'll move full speed until we hit our
    // MAX_SPEED_HALF_FRAME_SCALE position in the half frame.
    double scaledOutput = output / (((frameCenter / MAX_SPEED_HALF_FRAME_SCALE) * strafeController.getP()));
    // Square our output so we more quickly ramp to zero as we approach the center of our frame.
    strafeOutput = Utilities.squareValues(scaledOutput) * MAX_STRAFE_OUTPUT;
    strafeOutput = strafeOutput * 2.5;
    // Negative since our Pixy cam is on the back of our robot. Our
    // side-to-side values need to be inverted, since our side-to-side
    // values are relative to the front of the robot
    strafeOutput = MathUtil.clamp(
      -strafeOutput,
      -MAX_STRAFE_OUTPUT,
      MAX_STRAFE_OUTPUT
    );

    if (Math.abs(strafeOutput) < 0.08) {
      strafeOutput = Math.copySign(0.08, strafeOutput);
    }
    double straffedOutput = strafeOutput;
    SmartDashboard.putNumber("Strafe Output", straffedOutput);
    //strafeOutput = 0;
  }

  @Override
  public void end(boolean interrupted) {
    strafeOutput = 0;
    targetX = null;
    autoDrive.clearDelegate();
    // intake.stop();
    
  }

  @Override
  public State calculate(double forward, double strafe, boolean isFieldOriented) {
    if (targetX != null) {
      return new AutoDrive.State(
        spangle,
        strafeOutput
      );
    } else {
      return null;
    }
  }

  public boolean hasTarget() {
    return hasTarget;
  }

  public void changePipeline(int pipeline) {
    camera.setPipelineIndex(pipeline);
  }

  public double getYaw() {
    return yawValue;
  }

  public List<TargetCorner> getCorners() {
    return corners;
  }

  public double getFrameWidth() {
    //Find based off of PhotonVision output settings
    return 320;
  }

  public double getFrameCenter() {
    return getFrameWidth() / 2;
  }

  public Double getBallXValue() {
    return ballX;
  }

  private double getXValue(List<TargetCorner> targetCorners) {
    /**
     * PixyCam uses x-y coordinates to find balls and strafe, but PhotonVision only gives us rotations around axes for
     * finding our ball. However, getting the corner gives us a bounding box, which can be presumed to be pixels from
     * the camera output. So if we have a rectangle (the bounding box), we can get the x-values and divide them by two
     * to get the midpoint, and turn this midpoint of the ball to be relative to the camera center so that it can be
     * plugged into the PixyPickupCommand to run correctly.
     */


    //Hold Point values
    return (
      targetCorners.get(0) == targetCorners.get(1) ?          //Check if the first two are equal. There are only two unique values
      (targetCorners.get(0).x + targetCorners.get(2).x) / 2 : //If so, the first and third can be presumed to be unique. Use those
      (targetCorners.get(0).x + targetCorners.get(1).x) / 2   //Otherwise, resort to using the first two, which are different.
    );//If you don't like ternary, I can switch this out for a more readable if-statement
    
  }

}
