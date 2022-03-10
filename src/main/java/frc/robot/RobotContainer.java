// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.BallColor;
import frc.robot.commands.HeadingToTargetCommand;
import frc.robot.commands.auto.*;
import frc.robot.commands.climber.ClimberJoystickCommand;
import frc.robot.commands.delivery.BottomToTopCommand;
import frc.robot.commands.delivery.DeliveryOverrideCommand;
import frc.robot.commands.kicker.ForwardKickerCommand;
import frc.robot.commands.pixy.PixyPickupCommand;
import frc.robot.commands.pixy.PixyPickupCommand.PickupStrategy;
import frc.robot.commands.swerve.SwerveDriveCommand;
import frc.robot.nerdyfiles.oi.JoystickAnalogButton;
import frc.robot.nerdyfiles.oi.NerdyOperatorStation;
import frc.robot.commands.shooter.LinearShootCommand;
import frc.robot.commands.shooter.StartStopShooterCommand;
import frc.robot.commands.shooter.StopAllShooterSystemsCommand;
import frc.robot.commands.vision.InstantRelocalizeCommand;
import frc.robot.commands.vision.LimelightHeadingAndInstantRelocalizeCommand;
import frc.robot.commands.vision.PeriodicRelocalizeCommand;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final NerdyOperatorStation operatorStation = new NerdyOperatorStation(2);

  private final PigeonIMU pigeon = new PigeonIMU(0);
  private final PixyCam pixyCam = new PixyCam();

  private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Kicker kicker = new Kicker();
  private final AutoDrive autoDrive = new AutoDrive();
  private final Delivery delivery = new Delivery();
  private final Drivetrain drivetrain = new Drivetrain(pigeon);
  private final Vision vision = new Vision();
  private final Heading heading = new Heading(drivetrain::getGyroscopeRotation, drivetrain::isMoving);

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    drivetrain.setDefaultCommand(new SwerveDriveCommand(driverController, autoDrive, heading, drivetrain));
    heading.setDefaultCommand(new HeadingToTargetCommand(drivetrain::getTranslation, heading));
    vision.setDefaultCommand(new PeriodicRelocalizeCommand(drivetrain, vision));

    // Configure the button bindings
    configureButtonBindings();

    autonChooser.setDefaultOption("Do Nothing", new DoNothingCommand());
    autonChooser.addOption("Pos1 Left Two Ball", new Pos1LeftTwoBall(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));
    autonChooser.addOption("Pos1 Left R1 Punt D2 R2 Shoot", new Pos1LeftR1D2PR2(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));
    autonChooser.addOption("Pos1 Left Four Ball", new Pos1LeftR1D2PR2(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));

    autonChooser.addOption("Pos2 Middle Two Ball", new Pos2MidTwoBall(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));
    autonChooser.addOption("Pos2 Mid R2 Punt D2 R1 Shoot", new Pos2MidR2D2PR1(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));
    autonChooser.addOption("Pos2 Middle Four Ball", new Pos2MidFourBall(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));

    autonChooser.addOption("Pos3 Right Two Ball", new Pos3RightTwoBall(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));
    autonChooser.addOption("Pos3 Right Three Ball", new Pos3RightThreeBall(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));
    autonChooser.addOption("Pos3 Right Five Ball", new Pos3RightFiveBall(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));

    autonChooser.addOption("Test", new Test(autoDrive, delivery, drivetrain, heading));

    // Put it on SmartDashboard
    SmartDashboard.putData("AutonChooser", autonChooser);
    
    // Also put it on driver dashboard
    Constants.DRIVER_DASHBOARD.add("Auton Chooser", autonChooser)
      .withWidget(BuiltInWidgets.kComboBoxChooser)
      .withSize(6, 3)
      .withPosition(0, 0);
    
    // Put some more things on driver dashboard
    Constants.DRIVER_DASHBOARD.addBoolean("Alliance", () -> BallColor.getAllianceColor() == BallColor.RED)
      .withPosition(3, 6)
      .withSize(3, 3)
      .withProperties(Map.of("Color when true", "#ff3333", "Color when false", "#3333ff"));
  }

  public void resetRobot() {
    // Other option here is Constants.STARTING_ANGLE for booting against Hub
    pigeon.setYaw(0, 250);
    drivetrain.resetPosition(
      new Pose2d(
        Constants.Auto.kStartAtZero.toFieldCoordinate(),
        drivetrain.getGyroscopeRotation()
      )
    );
  }

  public void resetRobotTeleop() {
    pigeon.setYaw(0, 250);
    drivetrain.resetPosition(
      new Pose2d(
        Constants.Auto.kResetToZero.toFieldCoordinate(),
        drivetrain.getGyroscopeRotation()
      )
    );
  }

  public void resetRobotAuto() {
    pigeon.setYaw(Constants.STARTING_ANGLE, 250);
    drivetrain.resetPosition(new Pose2d(Constants.Auto.kPosition3RightStart.toFieldCoordinate(), drivetrain.getGyroscopeRotation()));
  }

  public void enableMaintainHeading() {
    heading.enableMaintainHeading();
  }

  private void configureButtonBindings() {
    /** Driver Controller */
    // Note: Left X + Y axis, Right X axis, and Left Bumper are used by SwerveDriveCommand
    JoystickButton driverX = new JoystickButton(driverController, XboxController.Button.kX.value);
    JoystickButton driverA = new JoystickButton(driverController, XboxController.Button.kA.value);
    JoystickButton driverB = new JoystickButton(driverController, XboxController.Button.kB.value);
    JoystickButton driverRightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
    JoystickAnalogButton driverTriggerLeft = new JoystickAnalogButton(driverController, XboxController.Axis.kLeftTrigger.value);
    JoystickAnalogButton driverTriggerRight = new JoystickAnalogButton(driverController, XboxController.Axis.kRightTrigger.value);
    JoystickButton driverBack = new JoystickButton(driverController, XboxController.Button.kBack.value);
    JoystickButton driverStart = new JoystickButton(driverController, XboxController.Button.kStart.value);

    driverA.whenPressed(heading::enableMaintainHeading);
    driverB.whileHeld(new StartStopShooterCommand(38.5, shooter));

    // driverLeftBumper.whenPressed(new PrepareShooterCommandGroup(BallColor.BLUE, delivery, kicker));
    // driverRightBumper.whenPressed(new PrepareShooterCommandGroup(BallColor.RED, delivery, kicker));

    driverTriggerLeft.whenHeld(new LinearShootCommand(19.25, delivery, kicker, shooter));
    driverTriggerRight.whenHeld(new LinearShootCommand(38.5, delivery, kicker, shooter));
    driverTriggerRight.whenReleased(new StopAllShooterSystemsCommand(delivery, kicker, shooter));
    driverTriggerLeft.whenReleased(new StopAllShooterSystemsCommand(delivery, kicker, shooter));

    driverBack.whenPressed(new InstantRelocalizeCommand(drivetrain, vision));
    driverX.whileHeld(new LimelightHeadingAndInstantRelocalizeCommand(drivetrain, heading, vision));

    /** Operator Controller * */
    // Note: Left X axis is used by DeliveryOverrideCommand
    JoystickButton operatorA = new JoystickButton(operatorController, XboxController.Button.kA.value);
    JoystickButton operatorB = new JoystickButton(operatorController, XboxController.Button.kB.value);
    JoystickButton operatorX = new JoystickButton(operatorController, XboxController.Button.kX.value);
    JoystickButton operatorRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    JoystickButton operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    JoystickAnalogButton operatorLeftTrigger = new JoystickAnalogButton(operatorController, XboxController.Axis.kLeftTrigger.value);
    JoystickAnalogButton operatorRightTrigger = new JoystickAnalogButton(operatorController, XboxController.Axis.kRightTrigger.value);
    Trigger operatorRightLeftBumper = operatorRightBumper.and(operatorLeftBumper);
    JoystickButton operatorBack = new JoystickButton(operatorController, XboxController.Button.kBack.value);
    JoystickButton operatorStart = new JoystickButton(operatorController, XboxController.Button.kStart.value);

    operatorA.whileHeld(new ForwardKickerCommand(kicker));

    operatorRightTrigger.whenPressed(intake::start, intake);
    operatorRightTrigger.whenReleased(intake::stop, intake);

    operatorLeftTrigger.whenPressed(intake::reverse, intake);
    operatorLeftTrigger.whenReleased(intake::stop, intake);

    operatorRightBumper.whileHeld(new PixyPickupCommand(PickupStrategy.RED, autoDrive, intake, pixyCam));
    operatorLeftBumper.whileHeld(new PixyPickupCommand(PickupStrategy.BLUE, autoDrive, intake, pixyCam));
    operatorRightLeftBumper.whenActive(new PixyPickupCommand(PickupStrategy.ANY, autoDrive, intake, pixyCam));

    operatorBack.whileHeld(new ClimberJoystickCommand(operatorController, climber));

    operatorB.whileHeld(new DeliveryOverrideCommand(operatorController, delivery));

    Trigger intakeBeamBreakTrigger = new Trigger(intake::getBeamBreakSensorStatus);
    intakeBeamBreakTrigger.whenInactive(new BottomToTopCommand(delivery));

    /** Driverstation Controls * */

    operatorStation.blueSwitch.whileHeld(new DeliveryOverrideCommand(operatorController, delivery));
  }


  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
