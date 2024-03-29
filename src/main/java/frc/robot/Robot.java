// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private String startingPos = "UnSet";
  private double startingAngle = 110.0;

  private RobotContainer m_robotContainer;
  private boolean autonomousRan = false;
  private int pigeonCounter = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    LiveWindow.disableAllTelemetry();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    m_robotContainer.resetRobot();
  }

  /*
  private void setupLogger() {
    Logger logger = Logger.getInstance();

    // Run as fast as possible during replay
    setUseTiming(isReal());
    // Log & replay "SmartDashboard" values (no tables are logged by default).
    LoggedNetworkTables.getInstance().addTable("/SmartDashboard");
    // Set a metadata value
    logger.recordMetadata("ProjectName", "2022Relentless");

    // Log to USB stick (name will be selected automatically)
    // logger.addDataReceiver(new ByteLogReceiver("/media/sda1/"));
    // Provide log data over the network, viewable in Advantage Scope.
    logger.addDataReceiver(new LogSocketServer(5800));

    // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.
    logger.start();
  }
  */

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("auto", autonomousRan);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.disableServos();
    pigeonCounter = 0;
  }

  @Override
  public void disabledPeriodic() {
    startingPos = m_robotContainer.getStartingPosition();
    SmartDashboard.putString("Starting Position", startingPos);
    startingAngle = m_robotContainer.getStartingAngle();
    SmartDashboard.putNumber("Starting Angle", startingAngle);

    /*
    if (m_robotContainer.getBlackButtonStatus()) {
      m_robotContainer.climberDisableBrakeMode();
    }
    */

    if (m_robotContainer.getPigeonState() != null) {
      if (pigeonCounter < 500) {
        pigeonCounter++;
      } else {
        if (m_robotContainer.hasActiveTarget()) {
          m_robotContainer.ledSetColor(Color.kRed, m_robotContainer.getTx());
        } else {
          m_robotContainer.setRGBGreen();
        }
      }
    } else {
      m_robotContainer.setLEDOff();
    }
  }
  
  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.setClimberSetpoint();
    m_robotContainer.resetRobotChooser(startingPos, startingAngle);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    autonomousRan = true;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_robotContainer.setClimberSetpoint();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.enableMaintainHeading();

    if (!autonomousRan) {
      m_robotContainer.resetRobotTeleop();
    }

    m_robotContainer.instantiateSubsystemsTeleop();
    m_robotContainer.configureButtonBindingsTeleop();
    m_robotContainer.stopAutoSubsystems();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
