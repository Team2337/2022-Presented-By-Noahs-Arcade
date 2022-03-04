package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hardware.ColorSensorTMD;

public class TestSubsystem extends SubsystemBase {

  private final ColorSensorTMD sensor = new ColorSensorTMD(Port.kOnboard);

  public TestSubsystem() {
    System.out.println("Hello world!");
  }

  @Override
  public void periodic() {
    Color color = sensor.getRawColor();
    SmartDashboard.putNumberArray("TMD colors", new double[]{
      color.blue,
      color.red,
      color.green
    });
    SmartDashboard.putNumber("TMD proximity", sensor.getProximity());
    SmartDashboard.putString("TMD given color", String.valueOf(sensor.getColor()));
  }
  
}
