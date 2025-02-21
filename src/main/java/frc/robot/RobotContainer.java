// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.pathplanner.lib.auto.NamedCommands;
//import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.Filesystem;
//import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopAlgaeIntakeCommand;
import frc.robot.commands.TeleopAlgaePivotCommand;
import frc.robot.commands.TeleopCoralCommand;
import frc.robot.commands.TeleopElevatorCommand;
import frc.robot.commands.TeleopPathCommand;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PathRunner;
import frc.robot.subsystems.SwerveSubsystem;
//import java.io.File;
import swervelib.SwerveInputStream;

//import frc.robot.Constants.OperatorConstants;

//import frc.robot.subsystems.SwerveSubsystem;
//import swervelib.SwerveInputStream;
//import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final Elevator elevator;
  private final Coral coral;
  private final AlgaeIntake algaeIntake;
  private final AlgaePivot algaePivot;

  private final PathRunner pathRunner = new PathRunner();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController = new XboxController(
      OperatorConstants.kDriverControllerPort);
  
  private final XboxController MechDriver;

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(DriveFieldOrientedAngularVelocity);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    this.MechDriver = new XboxController(1);
    this.elevator = new Elevator();
    this.coral = new Coral();
    this.algaeIntake = new AlgaeIntake();
    this.algaePivot = new AlgaePivot();

    configureCommands();
  }

  private void configureCommands() {

    pathRunner.setDefaultCommand(new TeleopPathCommand(
        pathRunner,
        () -> m_driverController.getAButton(),
        () -> m_driverController.getBButton(),
        () -> m_driverController.getXButton(),
        () -> m_driverController.getYButton()
      )
    );

    this.algaePivot.setDefaultCommand(new TeleopAlgaePivotCommand(
      this.algaePivot,
      () -> this.MechDriver.getLeftY()
    ));

    this.algaeIntake.setDefaultCommand(new TeleopAlgaeIntakeCommand(
      this.algaeIntake,
      () -> this.MechDriver.getRightBumperButton(),
      () -> this.MechDriver.getLeftBumperButton() 
      ));

    this.elevator.setDefaultCommand(new TeleopElevatorCommand(
            this.elevator,
            () -> this.MechDriver.getAButtonPressed(),
            () -> this.MechDriver.getBButtonPressed()
        ));

    this.coral.setDefaultCommand(new TeleopCoralCommand(
        this.coral,
        () -> this.MechDriver.getLeftTriggerAxis(),
        () -> this.MechDriver.getRightTriggerAxis()
      ));
    
  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1)
      .withControllerRotationAxis(m_driverController::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(m_driverController::getRightX,
          m_driverController::getRightY)
      .headingWhile(false);
  // affects the things
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  Command DriveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // pass;
  }

  /**
   * Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
   * () -> MathUtil.applyDeadband(m_driverController.getLeftY() * -1,
   * OperatorConstants.LEFT_Y_DEADBAND),
   * () -> MathUtil.applyDeadband(m_driverController.getLeftX() * -1,
   * OperatorConstants.LEFT_X_DEADBAND),
   * () -> m_driverController.getRightX() * -1);
   */

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
