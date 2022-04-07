package frc.robot.subsystems.hardware;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase  {
  
  private PhotonCamera camera = new PhotonCamera("intake");
  private PhotonPipelineResult targets;
  private PhotonTrackedTarget ball;
  private double yawValue = 0.0;
  private boolean hasTarget = false;
  private List<TargetCorner> corners;
  private Double ballX;

  public PhotonVision() {
  }

  @Override
  public void periodic() {
    targets = camera.getLatestResult();
    if (targets.hasTargets()) {
      ball = targets.getBestTarget();
      yawValue = ball.getYaw();
      corners = ball.getCorners();
      hasTarget = true;
      ballX = getXValue(corners);
      SmartDashboard.putNumber("Xvalue", ballX);
    }
    else {
      hasTarget = false;
      ballX = null;
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
    return 160;
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

    /* ArrayList<Double> ballXValues = new ArrayList<Double>();
    for (int i = 0; i < 4; i++) {
      //Get the point, start a count
      double point = Math.abs(targetCorners.get(i).x);
      double count = 0;
      //Add how many times the x value has been seen inside this array
      for (int z = 0; z < ballXValues.size(); z++) {
        if (ballXValues.get(z) != point){
          count++;
        }
      }
      //If it hasn't, it is unique and we want it.
      if (count == 0.0){
        ballXValues.add(point);
      }
    }
    //Get the overall "length" of the line segment
    double totalXValues = 0;
    for (int y = 0; y < ballXValues.size(); y++) {
      totalXValues = totalXValues + ballXValues.get(y);
    }
    //Divide by the size to get the x value the midpoint of ball is on.
    double xValue = totalXValues / ballXValues.size();
    return xValue; */
  }
}
