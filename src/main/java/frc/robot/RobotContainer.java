// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

//import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.XboxController;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveTeleopCommand;
import frc.robot.commands.TeleopAlgaeIntakeCommand;
import frc.robot.commands.TeleopAlgaePivotCommand;
import frc.robot.commands.TeleopCoralCommand;
import frc.robot.commands.TeleopElevatorCommand;
import frc.robot.commands.TeleopPathCommand;
import frc.robot.commands.VisionCommand;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PathRunner;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import swervelib.SwerveDrive;
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
  private final SwerveSubsystem drivebase;
  private final SwerveDrive swerveDrive;
  public final Vision vision;
  private final Elevator elevator;
  private final Coral coral;
  private final AlgaeIntake algaeIntake;
  private final AlgaePivot algaePivot;
  private final PathRunner pathRunner; //= new PathRunner();

  public final VisionCommand visionCommand;

  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController = new XboxController(
      OperatorConstants.kDriverControllerPort);
  
  private final XboxController MechDriver;

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    

    this.MechDriver = new XboxController(1);

    this.drivebase = new SwerveSubsystem();
    this.swerveDrive = drivebase.getSwerveDrive();

    this.pathRunner = new PathRunner();
    this.elevator = new Elevator();
    this.coral = new Coral();
    this.algaeIntake = new AlgaeIntake();
    this.algaePivot = new AlgaePivot();
    
    this.vision = new Vision(swerveDrive);

    visionCommand = new VisionCommand(vision);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();
    configureCommands();

    
    NamedCommands.registerCommand("ElevatorL1", Elevator.GetElevatorCommand(elevator, 0));
    NamedCommands.registerCommand("ElevatorL2", Elevator.GetElevatorCommand(elevator, 10));
    NamedCommands.registerCommand("ElevatorL3", Elevator.GetElevatorCommand(elevator, 20));
    NamedCommands.registerCommand("ElevatorL4", Elevator.GetElevatorCommand(elevator, 30));

    NamedCommands.registerCommand("CoralIn", Coral.GetCoralCommand(coral, 1, 1));
    NamedCommands.registerCommand("CoralOut", Coral.GetCoralCommand(coral, -1, 1));

    NamedCommands.registerCommand("AlgaeIntakeIn", AlgaeIntake.GetAlgaeIntakeCommand(algaeIntake, 1, 1));
    NamedCommands.registerCommand("AlgaeIntakeOut", AlgaeIntake.GetAlgaeIntakeCommand(algaeIntake, -1, 1));

    NamedCommands.registerCommand("AlgaePivotUp", AlgaePivot.GetAlgaePivotCommand(algaePivot, 0));
    NamedCommands.registerCommand("AlgaePivotDown", AlgaePivot.GetAlgaePivotCommand(algaePivot, 10));
  }

  

  private void configureCommands() {

    //new BooleanEvent(new EventLoop(), () -> m_driverController.getPOV() != -1).ifHigh(() -> System.out.println("DPAD PRESSED"));
    
    //this.m_driverController.x().onTrue(new InstantCommand(this.swerveDrive::zeroGyro, this.drivebase));
    
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
    
    drivebase.setDefaultCommand(new SwerveTeleopCommand(
      drivebase,
      swerveDrive,
      driveAngularVelocity,
      driveDirectAngle,
      () -> m_driverController.getPOV() != -1
    )); 


    pathRunner.setDefaultCommand(new TeleopPathCommand(
        pathRunner,
        () -> m_driverController.getAButton(),
        () -> m_driverController.getBButton(),
        () -> m_driverController.getXButton(),
        () -> m_driverController.getYButton()
      )
    );
    
    vision.setDefaultCommand(visionCommand);

    


    algaePivot.setDefaultCommand(new TeleopAlgaePivotCommand(
        algaePivot,
        () -> MechDriver.getLeftY()
      )
    );

    algaeIntake.setDefaultCommand(new TeleopAlgaeIntakeCommand(
        algaeIntake,
        () -> MechDriver.getRightBumperButton(),
        () -> MechDriver.getLeftBumperButton() 
      )
    );

    elevator.setDefaultCommand(new TeleopElevatorCommand(
            elevator,
            () -> MechDriver.getRightY()
          )
      );
    

    coral.setDefaultCommand(new TeleopCoralCommand(
        coral,
        () -> MechDriver.getLeftTriggerAxis(),
        () -> MechDriver.getRightTriggerAxis()
      )
    );
    
  }

  

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
      drivebase.setDefaultCommand(DriveFieldOrientedAngularVelocity);

    
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

  public Command getVisionCommand() {
    return visionCommand;
  }
}
