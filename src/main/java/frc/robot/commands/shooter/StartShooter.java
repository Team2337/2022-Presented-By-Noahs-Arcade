package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *Runs the shooter when a button is held and make sure it stops correctly
 *
 * @author Nicholas S.
 */
public class StartShooter extends CommandBase {

  private final Shooter shooter;
  private double reachSetpointTime;
  private boolean start = true;
  private double previousSetpoint;
  private double newSetpoint;

  public StartShooter(Shooter shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize(){
    reachSetpointTime = 0;
    start = true;
  }
  @Override
  public void execute() {
    newSetpoint = shooter.shooterSpeedFeetPerSecondWidget.getDouble(0);
    shooter.setSpeed(shooter.shooterSpeedFeetPerSecondWidget.getDouble(0));
    if (start || (previousSetpoint != newSetpoint)) {
      if (!shooter.isShooterToSpeed()) {
        reachSetpointTime ++;
      }
      else {
      start = false;
      SmartDashboard.putNumber("Time to reach Setpoint in sec", ((reachSetpointTime * 50) / 1000));
      }
    }
    previousSetpoint = shooter.shooterSpeedFeetPerSecondWidget.getDouble(0);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return shooter.isOverheated();
  }

}