package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.Status;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TimeOfFlightSensor extends SubsystemBase {
  
  private final TimeOfFlight sensor = new TimeOfFlight(0);
  private double previousDistance = 0.0;

  public TimeOfFlightSensor() {

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distance (in)", getDistanceInches());
    SmartDashboard.putString("Status", getStatus().toString());
  }

  public Status getStatus() {
    return sensor.getStatus();
  }

  public double getDistanceMM() {
    return sensor.getRange();
  }
  
  public double getDistanceInches() {
    if (getStatus() == Status.Valid){
      double distance = Units.metersToInches((getDistanceMM() / 1000));
      previousDistance = distance;
      return distance;
    }
    else {
      return previousDistance;
    }
  }
}
