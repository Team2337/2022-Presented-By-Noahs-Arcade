package frc.robot.commands.pixy;

import java.util.function.Supplier;

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
  private final PhotonVision photonVision;

  private final PIDController strafeController = new PIDController(0.0035, 0.0, 0.0); //P 0.0035

  // private var targetBall;
  private Double targetX = null;
  private int lastSeenCycleCounter = 0;
  private double strafeOutput = 0.0;
  private XboxController driverController;

  public Supplier<Rotation2d> gyroSupplier;

  public double spangle = 0;

  private PolarCoordinate joystickCoordinates = new PolarCoordinate(0, Rotation2d.fromDegrees(0));

  public PhotonPickupCommand(PickupStrategy strategy, Supplier<Rotation2d> gyroSupplier, XboxController driverController, AutoDrive autoDrive, PhotonVision photonVision) {
    this.strategy = strategy;
    this.gyroSupplier = gyroSupplier;
    this.driverController = driverController;
    this.autoDrive = autoDrive;
    this.photonVision = photonVision;

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
          photonVision.changePipeline(Vision.RED_PIPELINE_INDEX);
          break;
        case Blue:
          photonVision.changePipeline(Vision.BLUE_PIPELINE_INDEX);
          break;
        } 
      } else if (strategy == PickupStrategy.THEIRS) {
        switch (DriverStation.getAlliance()) {
          default:
          case Red:
            photonVision.changePipeline(Vision.BLUE_PIPELINE_INDEX);
            break;
          case Blue:
            photonVision.changePipeline(Vision.RED_PIPELINE_INDEX);
            break;
          } 
      }

    Double latestTargetX = photonVision.getBallXValue();
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

    double frameCenter = photonVision.getFrameCenter();
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

}
