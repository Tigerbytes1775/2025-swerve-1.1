package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import swervelib.SwerveDrive;

public class PathRunner extends SubsystemBase {

    

    private PathConstraints constraints;

    private Command goToFeeder1;
    private Pose2d feeder1Pose;

    private Command goToFeeder2;
    private Pose2d feeder2Pose;

    private Command goToProcessor;
    private Pose2d processorPose;

    private Command goToBarge;
    private Pose2d bargePose;

    private boolean isBlue = false;


    private Command currentCommand;


    public boolean commandRunning = false;
    public PathRunner() {
        congifurePaths();
    }

    

    public void stopPaths() {
        currentCommand.cancel();
    }

    public void goToFeeder1() {
        currentCommand = goToFeeder1;
        currentCommand.schedule();
    }

    public void goToFeeder2() {
        currentCommand = goToFeeder2;
        currentCommand.schedule();
    }

    public void goToProcessor() {
        currentCommand = goToProcessor;
        currentCommand.schedule();
    }

    public void goToBarge() {
        currentCommand = goToBarge;
        currentCommand.schedule();
    }

    private void congifurePaths() {


        
        DriverStation.getAlliance().ifPresent(
            alliance -> {
                isBlue = alliance == Alliance.Blue;
            }
        );


        constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        feeder1Pose = isBlue ? new Pose2d(1.229, 7.038, Rotation2d.fromDegrees(-55)) : new Pose2d(16.351, 7.048, Rotation2d.fromDegrees(-125));
        feeder2Pose = isBlue ? new Pose2d(1.16, 1.032, Rotation2d.fromDegrees(55)) : new Pose2d(16.302, 0.973, Rotation2d.fromDegrees(125));
        processorPose = isBlue ? new Pose2d(11.515, 7.515, Rotation2d.fromDegrees(90)) : new Pose2d(5.967 , 0.515, Rotation2d.fromDegrees(-90));
        bargePose = isBlue ? new Pose2d(8.834, 6.677, Rotation2d.fromDegrees(0)) : new Pose2d(8.804, 1.353, Rotation2d.fromDegrees(180));


        goToFeeder1 = AutoBuilder.pathfindToPose(
            feeder1Pose, 
            constraints, 
            0.0
        );

        goToFeeder2 = AutoBuilder.pathfindToPose(
            feeder2Pose, 
            constraints, 
            0.0
        );

        goToProcessor = AutoBuilder.pathfindToPose(
            processorPose, 
            constraints, 
            0.0
        );

        goToBarge = AutoBuilder.pathfindToPose(
            bargePose, 
            constraints, 
            0.0
        );
 
    }

    public Command GetTeleopCommand(XboxController controller) {
        return run(() -> {
            boolean aButton = controller.getAButton();
            boolean bButton = controller.getBButton();
            boolean xButton = controller.getXButton();  
            boolean yButton = controller.getYButton();  

            boolean anyButtonPressed = aButton || bButton || xButton || yButton;

            if(!anyButtonPressed && commandRunning) {
                stopPaths();
                commandRunning = false;
            }

            if(!commandRunning) {

                if(aButton) {
                    goToFeeder1();
                    commandRunning = true;
                } else if(bButton) {
                    goToFeeder2();
                    commandRunning = true;
                } else if(xButton) {
                    goToProcessor();
                    commandRunning = true;
                } else if(yButton) {
                    goToBarge();
                    commandRunning = true;
                } else {
                    commandRunning = false;
                }
            }
        });     
    }
}
