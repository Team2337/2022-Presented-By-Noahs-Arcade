// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.HeadingToTargetCommand;
import frc.robot.commands.auto.DoNothingCommand;
import frc.robot.commands.delivery.DeliveryOverrideCommand;
import frc.robot.commands.swerve.SwerveDriveCommand;
import frc.robot.nerdyfiles.Controller.JoystickAnalogButton;
import frc.robot.nerdyfiles.oi.NerdyOperatorStation;
import frc.robot.commands.shooter.RunKicker;
import frc.robot.commands.shooter.StartShooter;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final NerdyOperatorStation operatorStation = new NerdyOperatorStation(2);

  private final PigeonIMU pigeon = new PigeonIMU(0);
  private final PixyCam pixyCam = new PixyCam();

  // private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Kicker kicker = new Kicker();
  private final Vision vision = new Vision();
  private final AutoDrive autoDrive = new AutoDrive();
  private final Delivery delivery = new Delivery();
  private final Drivetrain drivetrain = new Drivetrain(pigeon);
  private final Heading heading = new Heading(drivetrain::getGyroscopeRotation, drivetrain::isMoving);

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    drivetrain.setDefaultCommand(new SwerveDriveCommand(driverController, autoDrive, heading, drivetrain));
    heading.setDefaultCommand(new HeadingToTargetCommand(drivetrain::getTranslation, heading));

    // Configure the button bindings
    configureButtonBindings();

    autonChooser.setDefaultOption("Do Nothing", new DoNothingCommand());

    SmartDashboard.putData("AutonChooser", autonChooser);
  }

  public void resetRobot() {
    pigeon.setYaw(0, 250);
  }

  private void configureButtonBindings() {
    /** Driver Controller */
    // Note: Left X + Y axis, Right X axis, and Left Bumper are used by SwerveDriveCommand
    JoystickButton driverX = new JoystickButton(driverController, XboxController.Button.kX.value);
    JoystickButton driverA = new JoystickButton(driverController, XboxController.Button.kA.value);
    JoystickButton driverB = new JoystickButton(driverController, XboxController.Button.kB.value);
    JoystickAnalogButton driverTriggerLeft = new JoystickAnalogButton(driverController, 2);
    JoystickAnalogButton driverTriggerRight = new JoystickAnalogButton(driverController, 3);

    driverX.whenPressed(heading::enableMaintainHeading);
    driverB.whileHeld(new StartShooter(shooter));
    driverTriggerRight.whileHeld(new RunKicker(kicker));

    /** Operator Controller * */
    // Note: Left X axis is used by DeliveryOverrideCommand
    JoystickButton operatorA = new JoystickButton(operatorController, XboxController.Button.kA.value);
    JoystickButton operatorB = new JoystickButton(operatorController, XboxController.Button.kB.value);
    JoystickButton operatorRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    JoystickButton operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    JoystickAnalogButton operatorTriggerLeft = new JoystickAnalogButton(operatorController, 2);
    JoystickAnalogButton operatorTriggerRight = new JoystickAnalogButton(operatorController, 3);

    operatorA.whenHeld(new StartShooter(shooter));
    operatorB.whenHeld(new RunKicker(kicker));
    
    operatorTriggerRight.whenPressed(intake::start, intake);
    operatorTriggerRight.whenReleased(intake::stop, intake);
    
    operatorRightBumper.whenPressed(intake::reverse, intake);
    operatorRightBumper.whenReleased(intake::stop, intake);
    
    // operatorX.whileHeld(new DeliveryOverrideCommand(operatorController, delivery));

    /** Driverstation Controls * */

    operatorStation.blueSwitch.whileHeld(new DeliveryOverrideCommand(operatorController, delivery));
  }


  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
