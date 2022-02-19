// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.BallColor;
import frc.robot.commands.HeadingToTargetCommand;
import frc.robot.commands.auto.DoNothingCommand;
import frc.robot.commands.delivery.DeliveryOverrideCommand;
import frc.robot.commands.delivery.IntakeBallCommandGroup;
import frc.robot.commands.delivery.PrepareShooterCommandGroup;
import frc.robot.commands.swerve.SwerveDriveCommand;
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

  public static BallColor allianceColor;
  public static BallColor opposingColor;
  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    drivetrain.setDefaultCommand(new SwerveDriveCommand(driverController, autoDrive, heading, drivetrain));
    heading.setDefaultCommand(new HeadingToTargetCommand(drivetrain::getTranslation, heading));

    // Set alliance color (this method gets called when robot is enabled, so set it then)
    allianceColor = DriverStation.getAlliance() == Alliance.Red ? BallColor.RED : BallColor.BLUE;
    opposingColor = allianceColor == BallColor.RED ? BallColor.BLUE : BallColor.RED;

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
    driverX.whenPressed(heading::enableMaintainHeading);

    /** Operator Controller * */
    // Note: Left X axis is used by DeliveryOverrideCommand
    JoystickButton operatorA = new JoystickButton(operatorController, XboxController.Button.kA.value);
    operatorA.whenHeld(new StartShooter(shooter));
    JoystickButton operatorB = new JoystickButton(operatorController, XboxController.Button.kB.value);
    operatorB.whenHeld(new RunKicker(kicker));
    JoystickButton operatorRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    operatorRightBumper.whenPressed(intake::startIntake, intake);
    operatorRightBumper.whenReleased(intake::stopIntake, intake);
    JoystickButton operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    operatorLeftBumper.whenPressed(intake::reverseIntake, intake);
    operatorLeftBumper.whenReleased(intake::stopIntake, intake);

    JoystickButton driverA = new JoystickButton(driverController, XboxController.Button.kA.value);
    driverA.whenPressed(new IntakeBallCommandGroup(intake, delivery));

    JoystickButton driverLeftBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    JoystickButton driverRightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
    driverLeftBumper.whenPressed(new PrepareShooterCommandGroup(BallColor.BLUE, delivery));
    driverRightBumper.whenPressed(new PrepareShooterCommandGroup(BallColor.RED, delivery));

    operatorStation.blueSwitch.whileHeld(new DeliveryOverrideCommand(operatorController, delivery));
  }


  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
