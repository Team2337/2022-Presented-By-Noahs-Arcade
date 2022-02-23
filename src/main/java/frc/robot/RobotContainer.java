// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.management.OperatingSystemMXBean;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.commands.delivery.commandgroups.*;
import frc.robot.commands.pixy.PixyPickupCommand;
import frc.robot.commands.pixy.PixyPickupCommand.PickupStrategy;
import frc.robot.commands.swerve.SwerveDriveCommand;
import frc.robot.nerdyfiles.oi.JoystickAnalogButton;
import frc.robot.nerdyfiles.oi.NerdyOperatorStation;
import frc.robot.commands.shooter.RecenterWithKicker;
import frc.robot.commands.shooter.ReverseKicker;
import frc.robot.commands.shooter.RunKicker;
import frc.robot.commands.shooter.StartShooter;
import frc.robot.commands.vision.InstantRelocalizeCommand;
import frc.robot.commands.vision.LimeLightHeadingCommand;
import frc.robot.commands.vision.PeriodicRelocalizeCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.hardware.PixyCam;
import frc.robot.subsystems.hardware.TimeOfFlightSensor;

public class RobotContainer {
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final NerdyOperatorStation operatorStation = new NerdyOperatorStation(2);

  private final PigeonIMU pigeon = new PigeonIMU(0);
  private final PixyCam pixyCam = new PixyCam();
  private final TimeOfFlightSensor TimeOfFlight = new TimeOfFlightSensor();

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
    // vision.setDefaultCommand(new PeriodicRelocalizeCommand(drivetrain, vision));

    // Configure the button bindings
    configureButtonBindings();

    autonChooser.setDefaultOption("Do Nothing", new DoNothingCommand());
    autonChooser.addOption("Pos1 Left One Ball", new Pos1LeftOneBall(autoDrive, drivetrain, heading));
    autonChooser.addOption("Pos1 Left R1 Punt D2 R2 Shoot", new Pos1LeftR1D2PR2(autoDrive, drivetrain, heading));
    autonChooser.addOption("Pos1 Left Four Ball", new Pos1LeftR1D2PR2(autoDrive, drivetrain, heading));

    autonChooser.addOption("Pos2 Middle One Ball", new Pos2MidOneBall(autoDrive, drivetrain, heading));
    autonChooser.addOption("Pos2 Mid R2 Punt D2 R1 Shoot", new Pos2MidR2D2PR1(autoDrive, drivetrain, heading));
    autonChooser.addOption("Pos2 Middle Four Ball", new Pos2MidFourBall(autoDrive, drivetrain, heading));

    autonChooser.addOption("Pos3 Right One Ball", new Pos3RightOneBall(autoDrive, drivetrain, heading));
    autonChooser.addOption("Pos3 Right Two Ball", new Pos3RightTwoBall(autoDrive, drivetrain, heading));
    autonChooser.addOption("Pos3 Right Five Ball", new Pos3RightFiveBall(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));

    autonChooser.addOption("Test", new Test(autoDrive, delivery, drivetrain, heading));
    autonChooser.addOption("Showcase Testing", new ShowcaseTesting(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));

    SmartDashboard.putData("AutonChooser", autonChooser);
  }

  public void resetRobot() {
    pigeon.setYaw(0, 250);
    drivetrain.resetPosition(
      new Pose2d(
        Constants.Auto.kStartAtZero.toFieldCoordinate(),
        drivetrain.getGyroscopeRotation()
      )
    );
  }

  public void resetRobot2() {
    pigeon.setYaw(60, 250);
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
    JoystickButton driverLeftBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    JoystickButton driverRightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
    JoystickAnalogButton driverTriggerLeft = new JoystickAnalogButton(driverController, 2);
    JoystickAnalogButton driverTriggerRight = new JoystickAnalogButton(driverController, 3);
    JoystickButton backButton = new JoystickButton(driverController, XboxController.Button.kBack.value);
    JoystickButton startButton = new JoystickButton(driverController, XboxController.Button.kStart.value);

    // driverA.whenPressed(delivery::resetArray);//TODO: debug; remove this before committing

    driverB.whileHeld(new StartShooter(shooter));
    driverX.whenPressed(heading::enableMaintainHeading);
    
    // driverLeftBumper.whenPressed(new PrepareShooterCommandGroup(BallColor.BLUE, delivery, kicker));
    // driverRightBumper.whenPressed(new PrepareShooterCommandGroup(BallColor.RED, delivery, kicker));
    driverTriggerRight.whileHeld(new RunKicker(kicker));
    
    backButton.whenPressed(new InstantRelocalizeCommand(drivetrain, vision));
    startButton.whileHeld(new LimeLightHeadingCommand(drivetrain, heading, vision));
    driverLeftBumper.whileHeld(new PixyPickupCommand(PickupStrategy.BLUE, autoDrive, pixyCam));
    driverRightBumper.whileHeld(new PixyPickupCommand(PickupStrategy.RED, autoDrive, pixyCam));


    /** Operator Controller * */
    // Note: Left X axis is used by DeliveryOverrideCommand
    JoystickButton operatorA = new JoystickButton(operatorController, XboxController.Button.kA.value);
    JoystickButton operatorB = new JoystickButton(operatorController, XboxController.Button.kB.value);
    JoystickButton operatorX = new JoystickButton(operatorController, XboxController.Button.kX.value);
    JoystickButton operatorY = new JoystickButton(operatorController, XboxController.Button.kY.value);
    JoystickButton operatorStart = new JoystickButton(operatorController, XboxController.Button.kStart.value);
    JoystickButton operatorRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    JoystickButton operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    JoystickAnalogButton operatorTriggerLeft = new JoystickAnalogButton(operatorController, 2);
    JoystickAnalogButton operatorTriggerRight = new JoystickAnalogButton(operatorController, 3);

    operatorStart.whenHeld(new StartShooter(shooter));
    operatorB.whenHeld(new RunKicker(kicker));
    operatorX.whileHeld(new DeliveryOverrideCommand(operatorController, delivery));
    operatorLeftBumper.whenPressed(new BottomToTopCommand(delivery, kicker));
    operatorY.whenPressed(new RecenterWithKicker(kicker, delivery));
    operatorTriggerRight.whenPressed(intake::start, intake);
    operatorTriggerRight.whenReleased(intake::stop, intake);
    
    operatorTriggerLeft.whileHeld(new ClimberJoystickCommand(operatorController, climber));
    
    operatorRightBumper.whenPressed(intake::reverse, intake);
    operatorRightBumper.whenReleased(intake::stop, intake);


    Trigger intakeBeamBreakTrigger = new Trigger(intake::getBeamBreakSensorStatus);
    //intakeBeamBreakTrigger.whenInactive(new AfterIntakeCommandGroup(intake, delivery, kicker));
    //intakeBeamBreakTrigger.whenInactive(new BottomToTopCommand(delivery, kicker));
    intakeBeamBreakTrigger.whenInactive(new RecenterWithKicker(kicker, delivery));
    /** Driverstation Controls * */

    operatorStation.blueSwitch.whileHeld(new DeliveryOverrideCommand(operatorController, delivery));
  }


  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
