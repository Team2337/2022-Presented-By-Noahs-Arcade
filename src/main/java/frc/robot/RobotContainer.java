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
import frc.robot.commands.HeadingToTargetCommand;
import frc.robot.commands.auto.*;
import frc.robot.commands.delivery.DeliveryOverrideCommand;
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
  private final Heading heading = new Heading(drivetrain::getGyroscopeRotation);

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    drivetrain.setDefaultCommand(new SwerveDriveCommand(driverController, autoDrive, heading, drivetrain));
    heading.setDefaultCommand(new HeadingToTargetCommand(() -> drivetrain.getPose().getTranslation(), heading));

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
    autonChooser.addOption("Pos3 Right Five Ball", new Pos3RightFiveBall(autoDrive, drivetrain, heading));

    autonChooser.addOption("Test", new Test(autoDrive, drivetrain, heading));

    SmartDashboard.putData("AutonChooser", autonChooser);
  }

  public void resetRobot() {
    /*
    if (DriverStation.getAlliance().toString() == "Red") {
      pigeon.setYaw(Constants.RED_STARTING_ANGLE, 250);
    } else {
      pigeon.setYaw(Constants.BLUE_STARTING_ANGLE, 250);
    }
    */
    pigeon.setYaw(25, 250);
    drivetrain.resetPosition(new Pose2d(Constants.Auto.kBallR2.toFieldCoordinate(), Rotation2d.fromDegrees(25)));
  }
  
  public void resetRobot2() {
    drivetrain.resetPosition(new Pose2d(Constants.Auto.kBallR2.toFieldCoordinate(), drivetrain.getGyroscopeRotation()));
  }

  public void enableMaintainHeading() {
    heading.enableMaintainHeading();
  }

  private void configureButtonBindings() {
    JoystickButton operatorA = new JoystickButton(operatorController, XboxController.Button.kA.value);
    operatorA.whenHeld(new StartShooter(shooter));
    JoystickButton operatorB = new JoystickButton(operatorController, XboxController.Button.kB.value);
    operatorB.whenHeld(new RunKicker(kicker));
    JoystickButton operatorX = new JoystickButton(operatorController, XboxController.Button.kX.value);
    JoystickButton driverX = new JoystickButton(driverController, XboxController.Button.kX.value);
    driverX.whenPressed(heading::enableMaintainHeading);

    // JoystickButton operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    JoystickButton operatorRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    operatorRightBumper.whenPressed(intake::start, intake);
    operatorRightBumper.whenReleased(intake::stop, intake);

    operatorStation.blueSwitch.whileHeld(new DeliveryOverrideCommand(operatorController, delivery));
    //operatorX.whileHeld(new DeliveryOverrideCommand(operatorController, delivery));
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
