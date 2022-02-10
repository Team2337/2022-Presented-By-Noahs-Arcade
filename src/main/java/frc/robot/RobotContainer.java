// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.*;
import frc.robot.commands.delivery.DeliveryOverrideCommand;
import frc.robot.commands.swerve.SwerveDriveCommand;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final NerdyOperatorStation operatorStation = new NerdyOperatorStation(2);

  private final PigeonIMU pigeon = new PigeonIMU(0);

  private final AutoDrive autoDrive = new AutoDrive();
  // private final Climber climber = new Climber();
  private final Delivery delivery = new Delivery();
  private final Drivetrain drivetrain = new Drivetrain(pigeon);
  private final Heading heading = new Heading(drivetrain::getGyroscopeRotation);
  private final Intake intake = new Intake();
  // private final Vision vision = new Vision();

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    drivetrain.setDefaultCommand(new SwerveDriveCommand(driverController, autoDrive, heading, drivetrain));

    // Configure the button bindings
    configureButtonBindings();

    autonChooser.setDefaultOption("Do Nothing", new DoNothingCommand());
    autonChooser.addOption("Top 3 Ball", new Top3Ball(autoDrive, drivetrain, heading));
    // autonChooser.addOption("Top 3 Ball", new Top3Ball(autoDrive, drivetrain, heading));
    autonChooser.addOption("Red Pos3 One Ball", new RedPos3OneBall(autoDrive, drivetrain, heading));
    // autonChooser.addOption("Red Pos2 One Ball", new RedPos2OneBall(autoDrive, drivetrain, heading));
    // autonChooser.addOption("Red Pos1 One Ball", new RedPos1OneBall(autoDrive, drivetrain, heading));
    // autonChooser.addOption("Red Pos3 Five Ball", new RedPos3FiveBall(autoDrive, drivetrain, heading));
    autonChooser.addOption("Blue Pos3 One Ball", new BluePos3OneBall(autoDrive, drivetrain, heading));

    SmartDashboard.putData("AutonChooser", autonChooser);
  }

  public void resetRobot() {
    if (DriverStation.getAlliance().toString() == "Red") {
      pigeon.setYaw(Constants.RED_STARTING_ANGLE, 250);
    } else {
      pigeon.setYaw(Constants.BLUE_STARTING_ANGLE, 250);
    }
    // pigeon.setYaw(0, 250);
    drivetrain.resetPosition(new Pose2d(Constants.Auto.bluePosition3Start.toFieldCoordinate(), Rotation2d.fromDegrees(0)));
  }

  public void resetRobot2() {
    drivetrain.resetPosition(new Pose2d(Constants.Auto.bluePosition3Start.toFieldCoordinate(), Rotation2d.fromDegrees(60)));
  }
  private void configureButtonBindings() {
    JoystickButton driverX = new JoystickButton(driverController, XboxController.Button.kX.value);
    driverX.whenPressed(heading::enableMaintainHeading);

    JoystickButton rightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
    JoystickButton leftBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    // leftBumper.whenPressed(new Top3Ball(drivetrain, heading, autoDrive));
    // leftBumper.whenPressed(new ProfiledPointToPointCommand(Constants.Auto.kBall1Pickup, drivetrain::getPose, drivetrain::getChassisSpeeds, heading, autoDrive));
    // rightBumper.whenPressed(new ProfiledPointToPointCommand(Constants.Auto.kBall2Pickup, drivetrain::getPose, drivetrain::getChassisSpeeds, heading, autoDrive));

    operatorStation.blueSwitch.whileHeld(new DeliveryOverrideCommand(operatorController, delivery));
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
