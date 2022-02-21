// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.DoNothingCommand;
import frc.robot.commands.climber.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);

  private final PigeonIMU pigeon = new PigeonIMU(0);
  //private final PixyCam pixyCam = new PixyCam();

  //private final Climber climber = new Climber();
  private final Climber climber = new Climber();

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    //drivetrain.setDefaultCommand(new SwerveDriveCommand(driverController, autoDrive, heading, drivetrain));

    // Configure the button bindings
    configureButtonBindings();

    autonChooser.setDefaultOption("Do Nothing", new DoNothingCommand());

    SmartDashboard.putData("AutonChooser", autonChooser);
    SmartDashboard.putData(climber);
  }

  public void resetRobot() {
    pigeon.setYaw(0, 250);
  }

  private void configureButtonBindings() {
    
    JoystickButton operatorStart = new JoystickButton(operatorController, XboxController.Button.kStart.value);
    JoystickButton operatorBack = new JoystickButton(operatorController, XboxController.Button.kBack.value);
    JoystickButton operatorA = new JoystickButton(operatorController, XboxController.Button.kA.value);
    JoystickButton operatorB = new JoystickButton(operatorController, XboxController.Button.kB.value);
    JoystickButton operatorX = new JoystickButton(operatorController, XboxController.Button.kX.value);
    JoystickButton operatorY = new JoystickButton(operatorController, XboxController.Button.kY.value);
    JoystickButton operatorRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    JoystickButton operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);


    operatorStart.whenPressed(new ClimberJoystickCommand(operatorController, climber));
    operatorStart.whenReleased(climber::stop);
    //operatorBack.whenPressed(climber::goLowRung);
    //operatorBack.whenReleased(climber::holdPositionUsingEncoder);
    operatorX.whenPressed(new ClimberSetPointCommand(climber.RICKABOOT, climber));
    operatorX.whenReleased(new ClimberSetPointCommand(climber.START, climber));
    //operatorY.whenPressed(new ClimberCommand(operatorController, Constants.Climber.LOW_RUNG, true, climber));

    // operatorRightBumper.whenPressed(intake::start, intake);
    // operatorRightBumper.whenReleased(intake::stop, intake);
    // operatorLeftBumper.whenPressed(intake::reverse, intake);
    // operatorLeftBumper.whenReleased(intake::stop, intake);
   
    // operatorStation.blueSwitch.whileHeld(new DeliveryOverrideCommand(operatorController, delivery));
    // operatorX.whileHeld(new DeliveryOverrideCommand(operatorController, delivery));
  }
  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
