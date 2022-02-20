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
  private double reachSetpointCycleCounter;
  private boolean isShooterComingUpToSpeed = true;
  private double previousSetpoint;
  private double newSetpoint;

  public StartShooter(Shooter shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize(){
    reachSetpointCycleCounter = 0;
    isShooterComingUpToSpeed = true;
  }
  @Override
  public void execute() {
    double targetSetpoint = shooter.shooterSpeedFeetPerSecondWidget.getDouble(0);
    newSetpoint = targetSetpoint;
    shooter.setSpeed(targetSetpoint);
    if (isShooterComingUpToSpeed || (previousSetpoint != newSetpoint)) {
      if (!shooter.isShooterToSpeed()) {
        reachSetpointCycleCounter ++;
      } else {
        isShooterComingUpToSpeed = false;
      }
    }
    previousSetpoint = targetSetpoint;
    //20 ms in a cycle, so we multiply our cycle time by 50 to get the amount of milliseconds, divide by 1000 to get seconds.
    SmartDashboard.putNumber("Time to reach Setpoint in sec", ((reachSetpointCycleCounter * 50) / 1000));
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