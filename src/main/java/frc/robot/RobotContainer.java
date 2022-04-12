// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.BallColor;
import frc.robot.Constants.DriverDashboardPositions;
import frc.robot.commands.HeadingToTargetCommand;
import frc.robot.commands.TriggerCommandGroup;
import frc.robot.commands.LED.*;
import frc.robot.commands.auto.*;
import frc.robot.commands.climber.ClimberDrive;
import frc.robot.commands.climber.ClimberJoystickCommand;
import frc.robot.commands.climber.ClimberSetpointCommand;
import frc.robot.commands.delivery.DeliveryOverrideCommand;
import frc.robot.commands.kicker.ForwardKickerCommand;
import frc.robot.commands.pixy.PhotonPickupCommand;
import frc.robot.commands.pixy.PhotonPickupCommand.PickupStrategy;
import frc.robot.commands.swerve.SwerveDriveCommand;
import frc.robot.nerdyfiles.oi.JoystickAnalogButton;
import frc.robot.nerdyfiles.oi.NerdyOperatorStation;
import frc.robot.commands.shooter.ConditionalAutomaticShoot;
import frc.robot.commands.shooter.OperatorLinearShootCommand;
import frc.robot.commands.shooter.PerpetualBloopOperatorLinearShoot;
import frc.robot.commands.shooter.PerpetualBloopShoot;
import frc.robot.commands.shooter.PerpetualConditionalBloopShoot;
import frc.robot.commands.shooter.PerpetualShoot;
import frc.robot.commands.shooter.PrepareShooter;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.StartStopShooterCommand;
import frc.robot.commands.shooter.StopAllShooterSystemsCommand;
import frc.robot.commands.vision.InstantRelocalizeCommand;
import frc.robot.commands.vision.LimelightHeadingAndInstantRelocalizeCommand;
import frc.robot.commands.vision.PeriodicRelocalizeCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.hardware.PixyCam;
import frc.robot.subsystems.hardware.LED;
import frc.robot.subsystems.hardware.PhotonVision;

public class RobotContainer {
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final NerdyOperatorStation operatorStation = new NerdyOperatorStation(2);

  private final PigeonIMU pigeon = new PigeonIMU(0);

  private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Kicker kicker = new Kicker();
  private final AutoDrive autoDrive = new AutoDrive();
  private final Delivery delivery = new Delivery();
  private final Drivetrain drivetrain = new Drivetrain(pigeon);
  private final Vision vision = new Vision();
  private final Heading heading = new Heading(drivetrain::getGyroscopeRotation, drivetrain::isMoving);
  private final LED LED = new LED();
  private final PixyCam pixyCam = new PixyCam();
  // private final PhotonVision photonVision = new PhotonVision();

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();
  private final SendableChooser<String> startingPosChooser = new SendableChooser<>();
  private final SendableChooser<Double> startingAngleChooser = new SendableChooser<>();

  public RobotContainer() {
    JoystickButton operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    JoystickButton driverRightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);

    drivetrain.setDefaultCommand(new SwerveDriveCommand(driverController, autoDrive, heading, drivetrain));
    heading.setDefaultCommand(
        new HeadingToTargetCommand(drivetrain::getTranslation, operatorLeftBumper::get, driverRightBumper::get, drivetrain, heading, vision));
    LED.setDefaultCommand(new LEDRunnable(LED, this));
    vision.setDefaultCommand(new PeriodicRelocalizeCommand(drivetrain, vision));
    climber.setDefaultCommand(new ClimberJoystickCommand(drivetrain::getGyroscopeRoll, operatorController, operatorStation, climber));

    // Configure the button bindings
    configureButtonBindings();

    // Create auton selector
    autonChooser.setDefaultOption("Do Nothing", new DoNothingCommand());
    autonChooser.addOption("Pos1 Left Two Ball",
        new Pos1LeftTwoBall(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));
    autonChooser.addOption("Pos1 Left R1 Punt D2 R2 Shoot",
        new Pos1LeftR1D2PR2(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));
    autonChooser.addOption("Pos1 Left Two Ball Partner One Ball",
        new Pos1LeftTwoBallPartnerOneBall(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));
    autonChooser.addOption("Pos1 Left Two Ball Rude One",
        new Pos1LeftTwoBallRudeOne(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));
    autonChooser.addOption("Pos1 Left Two Ball Rude One Prep R7",
        new Pos1LeftTwoBallRudeOnePrepR7(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));
    autonChooser.addOption("Pos1 Left Two Ball Rude Two",
        new Pos1LeftTwoBallRudeTwo(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));
    autonChooser.addOption("Pos1 Left Four Ball",
        new Pos1LeftFourBall(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));

    autonChooser.addOption("Pos2 Middle Two Ball",
        new Pos2MidTwoBall(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));
    autonChooser.addOption("Pos2 Mid R2 Punt D2 R1 Shoot",
        new Pos2MidR2D2PR1(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));
    autonChooser.addOption("Pos2 Middle Four Ball",
        new Pos2MidFourBall(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));
    autonChooser.addOption("Pos2 Middle Five Ball", 
        new Pos2MidFiveBall(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));

    autonChooser.addOption("Pos3 Right Two Ball",
        new Pos3RightTwoBall(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));
    autonChooser.addOption("Pos3 Right Three Ball",
        new Pos3RightThreeBall(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));
    autonChooser.addOption("Pos3 Right Five Ball",
        new Pos3RightFiveBall(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));
    autonChooser.addOption("Pos3 Right Five Ball v2",
        new Pos3RightFiveBallv2(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));
    autonChooser.addOption("Pos3 Right Five Ball Cheeseball",
        new Pos3RightFiveBallv3(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));

    autonChooser.addOption("Test", new Test(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter));

    SmartDashboard.putData("AutonChooser", autonChooser);

    startingPosChooser.setDefaultOption("Right Pos3", "Right");
    startingPosChooser.addOption("Left Pos1", "Left");
    startingPosChooser.addOption("Middle Pos2", "Middle");
    startingPosChooser.addOption("Far Right", "Far Right");

    SmartDashboard.putData("StartingPositionChooser", startingPosChooser);

    startingAngleChooser.addOption("Launchpad (0 degrees)", 0.0);
    startingAngleChooser.addOption("Left fender (-20 degrees)", -20.0);
    startingAngleChooser.addOption("Right fender (70 degrees)", 70.0);
    startingAngleChooser.setDefaultOption("Cargo exit (25 degrees)", 25.0);
    startingAngleChooser.addOption("Right Pos3 Errored Start (80 degrees)", 80.0);
    startingAngleChooser.addOption("Left Pos1 Errored Start (-35 degrees)", -35.0);
    startingAngleChooser.addOption("Middle Pos2 Errored Start (38 degrees)", 38.0);
    startingAngleChooser.addOption("Test (-35 degrees)", -35.0);

    SmartDashboard.putData("StartingAngleChooser", startingAngleChooser);

    // Add dropdowns to driver dashboard
    Constants.DRIVER_DASHBOARD.add("Auton Chooser", autonChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(DriverDashboardPositions.AUTON_CHOOSER.x, DriverDashboardPositions.AUTON_CHOOSER.y)
        .withSize(DriverDashboardPositions.AUTON_CHOOSER.width, DriverDashboardPositions.AUTON_CHOOSER.height);

    Constants.DRIVER_DASHBOARD.add("Starting Pos Chooser", startingPosChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(DriverDashboardPositions.STARTING_POS_CHOOSER.x, DriverDashboardPositions.STARTING_POS_CHOOSER.y)
        .withSize(DriverDashboardPositions.STARTING_POS_CHOOSER.width,
            DriverDashboardPositions.STARTING_POS_CHOOSER.height);

    Constants.DRIVER_DASHBOARD.add("Starting Angle Chooser", startingAngleChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(DriverDashboardPositions.STARTING_ANGLE_CHOOSER.x,
            DriverDashboardPositions.STARTING_ANGLE_CHOOSER.y)
        .withSize(DriverDashboardPositions.STARTING_ANGLE_CHOOSER.width,
            DriverDashboardPositions.STARTING_ANGLE_CHOOSER.height);

    // Put alliance on driver dashboard
    Constants.DRIVER_DASHBOARD.addBoolean("Alliance", () -> BallColor.getAllianceColor() == BallColor.Red)
        .withPosition(DriverDashboardPositions.ALLIANCE.x, DriverDashboardPositions.ALLIANCE.y)
        .withSize(3, 3)
        .withProperties(Map.of("Color when true", "#ff3333", "Color when false", "#3333ff"));

    /*
     * if (Constants.DO_SYSTEMS_CHECK) {
     * Constants.SYSTEMS_CHECK_TAB.addBoolean("Pixy Cam Connected",
     * pixyCam::isConnected)
     * .withPosition(SystemsCheckPositions.PIXY_CAM.x,
     * SystemsCheckPositions.PIXY_CAM.y)
     * .withSize(3, 3);
     * }
     */
  }

  public void resetRobot() {
    // Other option here is Constants.STARTING_ANGLE for booting against Hub
    pigeon.setYaw(0, 250);
    drivetrain.resetPosition(
        new Pose2d(
            Constants.Auto.kStartAtZero.toFieldCoordinate(),
            drivetrain.getGyroscopeRotation()));
  }

  public void resetRobotTeleop() {
    pigeon.setYaw(0, 250);
    drivetrain.resetPosition(
        new Pose2d(
            Constants.Auto.kResetToZero.toFieldCoordinate(),
            drivetrain.getGyroscopeRotation()));
  }

  public void resetRobotAuto() {
    pigeon.setYaw(-35, 250);
    drivetrain.resetPosition(
        new Pose2d(Constants.Auto.kPosition1LeftStart.toFieldCoordinate(), drivetrain.getGyroscopeRotation()));
  }

  public void resetRobotAuto(double startingAngle) {
    pigeon.setYaw(startingAngle + drivetrain.getGyroscopeRotation().getDegrees(), 250);
    drivetrain.resetPosition(
        new Pose2d(Constants.Auto.kPosition3RightStart.toFieldCoordinate(), drivetrain.getGyroscopeRotation()));
  }

  public void resetRobotChooser(String startPos, double startingAngle) {
    switch (startPos) {

      case "Left":
        pigeon.setYaw(startingAngle + drivetrain.getGyroscopeRotation().getDegrees(), 250); // -32.25 deg
        drivetrain.resetPosition(
            new Pose2d(Constants.Auto.kPosition1LeftStart.toFieldCoordinate(), drivetrain.getGyroscopeRotation()));
        break;

      case "Middle":
        pigeon.setYaw(startingAngle + drivetrain.getGyroscopeRotation().getDegrees(), 250); // 45 deg
        drivetrain.resetPosition(
            new Pose2d(Constants.Auto.kPosition2MiddleStart.toFieldCoordinate(), drivetrain.getGyroscopeRotation()));
        break;

      case "Right":
        pigeon.setYaw(startingAngle + drivetrain.getGyroscopeRotation().getDegrees(), 250); // 75 deg
        drivetrain.resetPosition(
            new Pose2d(Constants.Auto.kPosition3RightStart.toFieldCoordinate(), drivetrain.getGyroscopeRotation()));
        break;

      case "Far Right":
        pigeon.setYaw(startingAngle + drivetrain.getGyroscopeRotation().getDegrees(), 250); // 90 deg
        drivetrain.resetPosition(
            new Pose2d(Constants.Auto.kPositionFarRightStart.toFieldCoordinate(), drivetrain.getGyroscopeRotation()));
        break;

      default:
        pigeon.setYaw(startingAngle + drivetrain.getGyroscopeRotation().getDegrees(), 250); // 80 deg
        drivetrain.resetPosition(
            new Pose2d(Constants.Auto.kPosition3RightStart.toFieldCoordinate(), drivetrain.getGyroscopeRotation()));
        break;
    }
  }

  public void enableMaintainHeading() {
    heading.enableMaintainHeading();
  }

  public void disableServos() {
    climber.leftHookServo.setDisabled();
    climber.rightHookServo.setDisabled();
  }

  public void stopAutoSubsystems() {
    shooter.stop();
    kicker.stop();
  }

  private void configureButtonBindings() {
    /** Driver Controller */
    // Note: Left X + Y axis, Right X axis, and Left Bumper are used by
    // SwerveDriveCommand
    JoystickButton driverX = new JoystickButton(driverController, XboxController.Button.kX.value);
    JoystickButton driverA = new JoystickButton(driverController, XboxController.Button.kA.value);
    JoystickButton driverB = new JoystickButton(driverController, XboxController.Button.kB.value);
    JoystickButton driverY = new JoystickButton(driverController, XboxController.Button.kY.value);
    JoystickAnalogButton driverTriggerLeft = new JoystickAnalogButton(driverController,
        XboxController.Axis.kLeftTrigger.value);
    JoystickAnalogButton driverTriggerRight = new JoystickAnalogButton(driverController,
        XboxController.Axis.kRightTrigger.value);
    JoystickButton driverBack = new JoystickButton(driverController, XboxController.Button.kBack.value);
    JoystickButton driverStart = new JoystickButton(driverController, XboxController.Button.kStart.value);

    driverA.whenPressed(heading::enableMaintainHeading);
    driverB.whileHeld(new StartStopShooterCommand(38.5, shooter));

    // driverLeftBumper.whenPressed(new PrepareShooterCommandGroup(BallColor.BLUE,
    // delivery, kicker));
    // driverRightBumper.whenPressed(new PrepareShooterCommandGroup(BallColor.RED,
    // delivery, kicker));
    JoystickButton operatorY = new JoystickButton(operatorController, XboxController.Button.kY.value);

    driverTriggerRight.whenHeld(new Shoot(delivery, kicker));

    driverTriggerLeft.whenHeld(
        new OperatorLinearShootCommand(drivetrain::getTranslation, delivery, kicker, shooter));
    driverTriggerLeft.whenReleased(new StopAllShooterSystemsCommand(delivery, kicker, shooter));

    driverBack.whenPressed(new InstantRelocalizeCommand(drivetrain, vision));

    /** Operator Controller * */
    // Note: Left X axis is used by DeliveryOverrideCommand

    JoystickButton operatorA = new JoystickButton(operatorController, XboxController.Button.kA.value);
    JoystickButton operatorB = new JoystickButton(operatorController, XboxController.Button.kB.value);
    JoystickButton operatorX = new JoystickButton(operatorController, XboxController.Button.kX.value);
    JoystickButton operatorRightStick = new JoystickButton(operatorController, XboxController.Button.kRightStick.value);
    JoystickButton operatorLeftStick = new JoystickButton(operatorController, XboxController.Button.kLeftStick.value);
    JoystickButton operatorRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);

    // Operator left bumper used for vision tracking by default commands.
    // Operator right bumper below in the configureButtonBindingsTeleop() method.
    JoystickAnalogButton operatorLeftTrigger = new JoystickAnalogButton(operatorController, XboxController.Axis.kLeftTrigger.value);
    JoystickAnalogButton operatorRightTrigger = new JoystickAnalogButton(operatorController, XboxController.Axis.kRightTrigger.value);
    JoystickButton operatorBack = new JoystickButton(operatorController, XboxController.Button.kBack.value);
    JoystickButton operatorStart = new JoystickButton(operatorController, XboxController.Button.kStart.value);

    JoystickButton yellowSwitch = new JoystickButton(operatorStation, 4);
    JoystickButton yellowButton = new JoystickButton(operatorStation, 8);
    JoystickButton blueButton = new JoystickButton(operatorStation, 9);

    operatorA.whileHeld(new ForwardKickerCommand(kicker));

    operatorRightTrigger.whenPressed(intake::start, intake);
    operatorRightTrigger.whenReleased(intake::stop, intake);

    operatorLeftTrigger.whenPressed(intake::reverse, intake);
    operatorLeftTrigger.whenReleased(intake::stop, intake);

    // operatorStart.whileHeld(new PhotonPickupCommand(PickupStrategy.OURS, drivetrain::getGyroscopeRotation, driverController, autoDrive, photonVision));
    // operatorBack.whileHeld(new PhotonPickupCommand(PickupStrategy.THEIRS, drivetrain::getGyroscopeRotation, driverController, autoDrive, photonVision));

    operatorRightBumper.whenHeld(new PrepareShooter(drivetrain::getTranslation, operatorY::get, vision::calculateDistanceToTargetInches, this::getClearSwitchStatus, kicker, shooter));
    operatorRightBumper.whenReleased(new StopAllShooterSystemsCommand(delivery, kicker, shooter));

    operatorRightStick.whileHeld(new LimelightHeadingAndInstantRelocalizeCommand(drivetrain, heading, vision));
    operatorLeftStick.whenPressed(new ClimberSetpointCommand(climber.RICKABOOT, climber));
    operatorLeftStick.whenReleased(new ClimberSetpointCommand(climber.TRAVEL_LOCATION, climber).withTimeout(1));

    // yellowSwitch.whenHeld(new ClimberSetpointWithTimeout(climber.MID_RUNG, climber).withTimeout(2.5));
    yellowButton.whileHeld(new ClimberDrive(1.0, drivetrain::getGyroscopeRoll, operatorStation, climber));
    blueButton.whileHeld(new ClimberDrive(-1.0, drivetrain::getGyroscopeRoll, operatorStation, climber));

    operatorB.whileHeld(new DeliveryOverrideCommand(operatorController, delivery));

    operatorX.whileHeld(new OperatorLinearShootCommand(drivetrain::getTranslation, delivery, kicker, shooter));
    operatorX.whenReleased(new StopAllShooterSystemsCommand(delivery, kicker, shooter));

    /** Driverstation Controls * */

    // operatorStation.blueSwitch.whileHeld(new
    // DeliveryOverrideCommand(operatorController, delivery));
  }

  public void instantiateSubsystemsTeleop() {
    // pixyCam = new PixyCam();
    System.out.println("Hello World!");
  }

  public void configureButtonBindingsTeleop() {
    JoystickButton redLeftSwitch = new JoystickButton(operatorStation, 11);

    // Trigger operatorRightLeftBumper =
    // operatorRightBumper.and(operatorLeftBumper);
    Trigger intakeBeamBreakTrigger = new Trigger(intake::getBeamBreakSensorStatus);
    intakeBeamBreakTrigger.whenInactive(new TriggerCommandGroup(shooter::isShooterToSpeed, delivery::getLeftColorSensorAllianceBallColor, driverController, delivery).andThen(new WaitCommand(0.3)).andThen(new PerpetualConditionalBloopShoot(redLeftSwitch::get, climber, delivery, kicker, shooter).andThen(new StopAllShooterSystemsCommand(delivery, kicker, shooter))));

    Trigger shootTrigger = new Trigger(() -> robotLinedUp());
    shootTrigger.whenActive(new ConditionalAutomaticShoot(redLeftSwitch::get, climber, delivery, kicker, shooter));
  }

  public boolean robotLinedUp() {
    // Idea behind this is that we use the onTarget and shooterToSpeed methods to
    // know we are on target, not just seeing it,
    // and we are up to speed, and we are not driving (joystick not touched), so we
    // are not shooting while on the move,
    // at least not yet, and we are waiting to stop and then immediately shooting.
    return (isOnTarget()
        && hasActiveTarget()
        && isShooterUpToLEDSpeed()
        && !drivetrain.isMoving()
        && operatorController.getRightBumper()
        && operatorController.getLeftBumper())
        && !operatorStation.redRightSwitch.get();
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }

  public String getStartingPosition() {
    return startingPosChooser.getSelected();
  }

  public double getStartingAngle() {
    return startingAngleChooser.getSelected();
  }

  public boolean getBlackSwitchStatus() {
    return operatorStation.blackSwitch.get();
  }

  public boolean getYellowSwitchStatus() {
    return operatorStation.yellowSwitch.get();
  }

  public boolean getClearSwitchStatus() {
    return operatorStation.clearSwitch.get();
  }

  public boolean getOperatorBackStatus() {
    return operatorController.getBackButton();
  }

  public boolean getOperatorStartStatus() {
    return operatorController.getStartButton();
  }

  public double getGyroscopeRotation() {
    return drivetrain.getGyroscopeRotation().getDegrees();
  }

  public double getGyroscopeRoll() {
    return drivetrain.getGyroscopeRoll().getDegrees();
  }

  public void setLEDColor(Color color) {
    LED.setColor(color);
  }

  public void setLEDOff() {
    LED.setOff();
  }

  public PigeonState getPigeonState() {
    return drivetrain.getPigeonState();
  }

  public boolean isShooterUpToLEDSpeed() {
    return shooter.isShooterToLEDSpeed();
  }

  public boolean isOnTarget() {
    return vision.isOnTarget();
  }

  public boolean hasActiveTarget() {
    return vision.hasActiveTarget();
  }

  public double getTx() {
    return vision.getTx();
  }

  public double getFrameCenter() {
    return pixyCam.getFrameCenter();
  }

  public void ledSetColor(Color color, double tx) {
    LED.setColor(color, tx);
  }

  public void climberDisableBrakeMode() {
    climber.disableBrakeMode();
  }

  public boolean getOperatorRightTriggerStatus() {
    return operatorController.getRightTriggerAxis() > 0.5;
  }

  public void setRGBGreen() {
    LED.setColorRGB(0, 25, 0);
  }

  public boolean getIntakeSensorStatus() {
    return intake.getBeamBreakSensorStatus();
  }

  public boolean getDriverRightBumperStatus() {
    return driverController.getRightBumper();
  }

  public void setClimberSetpoint() {
    climber.setPosition(climber.TRAVEL_LOCATION);
  }
}
