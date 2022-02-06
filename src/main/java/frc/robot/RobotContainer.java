// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.DoNothingCommand;
import frc.robot.commands.climber.ClimberCommand;
import frc.robot.commands.swerve.SwerveDriveCommand;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);

  private final PigeonIMU pigeon = new PigeonIMU(0);

  private final Climber climber = new Climber();
  // private final Delivery delivery = new Delivery();
  private final Drivetrain drivetrain = new Drivetrain(pigeon);
  private final Heading heading = new Heading(drivetrain::getGyroscopeRotation);
  // private final Intake intake = new Intake();
  // private final Vision vision = new Vision();

  public RobotContainer() {
    drivetrain.setDefaultCommand(new SwerveDriveCommand(driverController, heading, drivetrain));
    
    // Configure the button bindings
    configureButtonBindings();
  }

  public void resetGyro() {
    pigeon.setYaw(0, 250);
  }

  private void configureButtonBindings() {
    JoystickButton driverX = new JoystickButton(driverController, XboxController.Button.kX.value);
    driverX.whenPressed(heading::setNextHeadingToMaintainHeading);

    //True means that the stringpot will be used for movement, otherwise, it is false
    JoystickButton operatorA = new JoystickButton(operatorController, XboxController.Button.kA.value);
    operatorA.whenHeld(new ClimberCommand(climber, operatorController, -0.3, false));
    JoystickButton operatorB = new JoystickButton(operatorController, XboxController.Button.kB.value);
    operatorB.whenHeld(new ClimberCommand(climber, operatorController, 1, true));
    JoystickButton operatorX = new JoystickButton(operatorController, XboxController.Button.kX.value);
    operatorX.whenHeld(new ClimberCommand(climber, operatorController, 0.104, true));
  }

  public Command getAutonomousCommand() {
    return new DoNothingCommand();
  }
}
