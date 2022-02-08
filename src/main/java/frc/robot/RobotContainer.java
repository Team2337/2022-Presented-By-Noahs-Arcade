// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.DoNothingCommand;
import frc.robot.commands.auto.Top3Ball;
import frc.robot.commands.interfaces.PosableAuton;
import frc.robot.commands.swerve.SwerveDriveCommand;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);

  private final PigeonIMU pigeon = new PigeonIMU(0);

  // private final Climber climber = new Climber();
  // private final Delivery delivery = new Delivery();
  private final AutoDrive autoDrive = new AutoDrive();
  private final Drivetrain drivetrain = new Drivetrain(pigeon);
  private final Heading heading = new Heading(drivetrain::getGyroscopeRotation);
  // private final Intake intake = new Intake();
  // private final Vision vision = new Vision();

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    drivetrain.setDefaultCommand(new SwerveDriveCommand(driverController, autoDrive, heading, drivetrain));

    // Configure the button bindings
    configureButtonBindings();

    autonChooser.setDefaultOption("Do Nothing", new DoNothingCommand());
    autonChooser.addOption("Top 3 Ball", new Top3Ball(autoDrive, drivetrain, heading));

    SmartDashboard.putData("AutonChooser", autonChooser);
  }

  public void resetRobot() {
    pigeon.setYaw(0, 250);
  }

  private void configureButtonBindings() {
    JoystickButton driverX = new JoystickButton(driverController, XboxController.Button.kX.value);
    JoystickButton rightTrigger = new JoystickButton(driverController, XboxController.Axis.kRightTrigger.value);
    JoystickButton rightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
    JoystickButton leftBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    // driverX.whenPressed(heading::setNextHeadingToMaintainHeading);
    // Note: Set to maintain a distance from 0, 0 - needs to be dropped once we're on the field
    // Maintain 1 ft distance in front of target
    // rightBumper.whileHeld(new DistanceToTargetCommand(Units.feetToMeters(1), drivetrain::getPose, drivetrain::getChassisSpeeds, heading, autoDrive));
    // leftBumper.whileHeld(new PointToPointCommand(Constants.Auto.kBall1, drivetrain::getPose, drivetrain::getChassisSpeeds, heading, autoDrive));
    // leftBumper.whenPressed(new Top3Ball(drivetrain, heading, autoDrive));
    //leftBumper.whenPressed(new ProfiledPointToPointCommand(Constants.Auto.kBall1Pickup, drivetrain::getPose, drivetrain::getChassisSpeeds, heading, autoDrive));
    //rightBumper.whenPressed(new ProfiledPointToPointCommand(Constants.Auto.kBall2Pickup, drivetrain::getPose, drivetrain::getChassisSpeeds, heading, autoDrive));
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }

  public void updateAutonStartPose() {
    Command autonCommand = autonChooser.getSelected();
    if (autonCommand instanceof PosableAuton) {
      PosableAuton poseableCommand = (PosableAuton)autonCommand;
      drivetrain.resetPosition(poseableCommand.getStartingPose());
    } else {
      drivetrain.resetPosition(new Pose2d(0, 0, drivetrain.getGyroscopeRotation()));
    }
  }
}
