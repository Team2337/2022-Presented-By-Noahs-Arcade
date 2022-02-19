package frc.robot.commands.pixy;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.interfaces.AutoDrivableCommand;
import frc.robot.subsystems.AutoDrive.State;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.PixyCam;

public class PixyPickupCommand extends CommandBase implements AutoDrivableCommand {
  
  /**
   * Whatever ball color we want to pick up. Red, blue, or any.
   */
  public static enum PickupStrategy {
    RED,
    BLUE,
    ANY
  }
  
  private final PickupStrategy strategy;
  private final AutoDrive autoDrive;
  private final PixyCam pixyCam;
  
  private PIDController strafeController = new PIDController(0.0035, 0.0, 0.0);
  
  private static final double kMaxStrafeOutput = 0.4;
  private static final double LAST_SEEN_COUNTER_MAX = 100; // 2 seconds
  private Block targetBall;
  private int lastSeenCounter = 0;
  private double strafeOutput = 0.0;

  public PixyPickupCommand(PickupStrategy strategy, AutoDrive autoDrive, PixyCam pixyCam) {
    this.strategy = strategy;
    this.pixyCam = pixyCam;
    this.autoDrive = autoDrive;

    addRequirements(autoDrive);
  }

  @Override
  public void initialize() {
    targetBall = null;
    lastSeenCounter = 0;
    autoDrive.setDelegate(this);
  }

  @Override
  public void execute() {
    Block latestTargetBall = null;
    if (strategy == PickupStrategy.ANY) {
      switch (DriverStation.getAlliance()) {
        default:
        case Red:
          // If alliance is red, prioritize red targets if they are there; otherwise get
          // blue targets
          latestTargetBall = pixyCam.getRedTarget() == null ? pixyCam.getBlueTarget() : pixyCam.getRedTarget();
          break;
        case Blue:
          // If alliance is blue, prioritize blue targets if they are there; otherwise get
          // red targets
          latestTargetBall = pixyCam.getBlueTarget() == null ? pixyCam.getRedTarget() : pixyCam.getBlueTarget();
          break;
      }
    } else if (strategy == PickupStrategy.RED) {
      latestTargetBall = pixyCam.getRedTarget();
    } else if (strategy == PickupStrategy.BLUE) {
      latestTargetBall = pixyCam.getBlueTarget();
    }
    Logger.getInstance().recordOutput("PixyPickup/Counter", lastSeenCounter);
    
    if (targetBall != null && latestTargetBall == null) {
      lastSeenCounter++;
    } else {
      lastSeenCounter = 0;
      targetBall = latestTargetBall;
    } 
    if (lastSeenCounter >= LAST_SEEN_COUNTER_MAX) {
      targetBall = null;
    }

    strafeOutput = 0.0;

    if (targetBall == null) {
      return;
    }
    
    double frameCenter = pixyCam.getFrameWidth() / 2.0;
    double output = strafeController.calculate(
      (double) targetBall.getX(),
      frameCenter
    );

    strafeOutput = (output / ((frameCenter / 1) * strafeController.getP())) * kMaxStrafeOutput;

    // Negative since our Pixy cam is on the back of our robot. Our
    // side-to-side values need to be inverted, since our side-to-side
    // values are relative to the front of the robot
    strafeOutput = -strafeOutput;
    strafeOutput = MathUtil.clamp(strafeOutput, -kMaxStrafeOutput, kMaxStrafeOutput);

    SmartDashboard.putNumber("PixyPickup/Strafe Output", strafeOutput);
    Logger.getInstance().recordOutput("PixyPickup/Strafe Output", strafeOutput);
    Logger.getInstance().recordOutput("PixyPickup/Controller Error", strafeController.getPositionError());
  }

  @Override
  public void end(boolean interrupted) {
    autoDrive.clearDelegate();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public State calculate(double forward, double strafe, boolean isFieldOriented) {
    if (targetBall != null) {
      // TODO: This ONLY works if the driver is in robot oriented... not good.
      return new AutoDrive.State(
        forward,
        strafeOutput
      );
    } else {
      return null;
    }
  }

}
