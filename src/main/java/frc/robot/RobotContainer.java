// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CoralAutoCommand;
import frc.robot.commands.ElevatorAutoCommand;
import frc.robot.commands.ElevatorTeleopCommand;
import frc.robot.commands.VisionTeleopCommand;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
//import frc.robot.subsystems.PathRunner;
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
  private final SwerveSubsystem swerveSubsystem;
  private final SwerveDrive swerveDrive;
  public final Vision vision;
  private final Elevator elevator;
  private final Coral coral;
  private final AlgaeIntake algaeIntake;
  private final AlgaePivot algaePivot;
  //private final PathRunner pathRunner; //= new PathRunner();



  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController driverController = new XboxController(
      OperatorConstants.kDriverControllerPort);
  
  private final XboxController MechDriver;

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    

    this.MechDriver = new XboxController(1);

    
    
    this.elevator = new Elevator();
    this.coral = new Coral();
    this.algaeIntake = new AlgaeIntake();
    this.algaePivot = new AlgaePivot();
    
    


    NamedCommands.registerCommand("ElevatorL1", new ElevatorAutoCommand(elevator, elevator.L1Height - 1, 1.5));
    NamedCommands.registerCommand("ElevatorL2",new ElevatorAutoCommand(elevator, elevator.L2Height, 2));
    NamedCommands.registerCommand("ElevatorL3", new ElevatorAutoCommand(elevator, elevator.L3Height, 2));
    NamedCommands.registerCommand("ElevatorL4", new ElevatorAutoCommand(elevator, elevator.L4Height, 2));

    NamedCommands.registerCommand("CoralIn", new CoralAutoCommand(coral, false, 1, 3));
    NamedCommands.registerCommand("CoralOut", new CoralAutoCommand(coral, true, 1, 0.5));
    NamedCommands.registerCommand("CoralOutBack", new CoralAutoCommand(coral, true, -0.125, 0.26));

    NamedCommands.registerCommand("AlgaeIntakeIn", AlgaeIntake.GetAlgaeIntakeCommand(algaeIntake, 1, 1));
    NamedCommands.registerCommand("AlgaeIntakeOut", AlgaeIntake.GetAlgaeIntakeCommand(algaeIntake, -1, 1));

    NamedCommands.registerCommand("AlgaePivotUp", AlgaePivot.GetAlgaePivotCommand(algaePivot, 0));
    NamedCommands.registerCommand("AlgaePivotDown", AlgaePivot.GetAlgaePivotCommand(algaePivot, 10));

    this.swerveSubsystem = new SwerveSubsystem();
    this.swerveDrive = swerveSubsystem.getSwerveDrive();

    //this.pathRunner = new PathRunner(swerveSubsystem);
    this.vision = new Vision(swerveDrive);


    

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();
    configureCommands();

    
    //NamedCommands.registerCommand("Print", new Command.PrintCommand());

    
  }

  

  private void configureCommands() {

    //pathRunner.setDefaultCommand(
    //  pathRunner.GetTeleopCommand(driverController)
    //);
    
    vision.setDefaultCommand(new VisionTeleopCommand(
        vision, 
        () -> driverController.getPOV() != -1
      )
    );
    
    algaePivot.setDefaultCommand(
      algaePivot.GetTeleopCommand(MechDriver)
    );

    algaeIntake.setDefaultCommand(
      algaeIntake.GetTeleopCommand(MechDriver)
    );

    elevator.setDefaultCommand(new ElevatorTeleopCommand(
        elevator, 
        MechDriver
      )
    );    

    coral.setDefaultCommand(
      coral.GetTeleopCommand(MechDriver)
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
    
    DoubleSupplier swerveScalar = () -> driverController.getLeftBumperButton() || driverController.getRightBumperButton() ? 0.5 : 1.0;
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      swerveSubsystem.getSwerveDrive(),
      () -> driverController.getLeftY() * -swerveScalar.getAsDouble(),
      () -> driverController.getLeftX() * -swerveScalar.getAsDouble()
    )
      .withControllerRotationAxis(() -> -driverController.getRightX() * swerveScalar.getAsDouble())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)//####################################driver 1 deadband##################################3
      .allianceRelativeControl(true);


      
    //SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
    //  .withControllerHeadingAxis(driverController::getRightX,
    //      driverController::getRightY)
    //  .headingWhile(false);
    //// affects the things
    //Command driveFieldOrientedDirectAngle = swerveSubsystem.driveFieldOriented(driveDirectAngle);

    Command DriveFieldOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(
      driveAngularVelocity, 
      () -> driverController.getPOV() != -1
    );
    swerveSubsystem.setDefaultCommand(DriveFieldOrientedAngularVelocity);
    
    
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
