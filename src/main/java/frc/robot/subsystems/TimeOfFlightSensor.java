package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
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
    SmartDashboard.putString("Status", getStatus());
  }

  public String getStatus() {
    return sensor.getStatus().toString();
  }

  public double getDistanceMM() {
    return sensor.getRange();
  }

  public double getDistanceInches() {
    if (getStatus().equals("Valid")){
      previousDistance = (sensor.getRange() / 25.4);
      return sensor.getRange() / 25.4; // mm to in
    }
    else {
      return previousDistance;
    }
  }
}
