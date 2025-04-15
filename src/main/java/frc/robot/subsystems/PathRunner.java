package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import swervelib.SwerveDrive;

public class PathRunner extends SubsystemBase {

    private double[][] allBlueReefInfo = new double[][]{
        {4.914, 3.03, -55},
        {5.207, 3.138, -55},
        {5.587, 3.84, 0},
        {5.606, 4.191, 0},
        {5.187, 4.883, 58},
        {4.895, 5.049, 58},
        {4.066, 5.058, 120},
        {3.783, 4.912, 120},
        {3.393, 4.191, 180},
        {3.383, 3.869, 180},
        {3.812, 3.138, -120},
        {4.095, 2.992, -120}
      //  {3.7,2.94,60},
      //  {3.98,2.78,60},
      //  {3.2,4.19,0},
      //  {3.2,3.85,0},
      //  {3.96,5.28,-60},
      //  {3.68,5.13,-60},
      //  {5.29,5.11,-120},
      //  {5.02,5.29,-120},
      //  {5.81,3.84,180},
      //  {5.81,4.17,180},
      //  {5.01,2.8,120},
      //  {5.29,2.95,120}
    };

    private double[][] allRedReefInfo = new double[][]{
        {12.392, 3.157,	-120},
        {12.685, 3.011,	-120},
        {11.983, 4.191,	180},
        {11.992, 3.849,	180},
        {12.665, 5.078,	120},
        {12.383, 4.912,	120},
        {13.787, 4.902,	60},
        {13.445, 5.029,	60},
        {14.167, 3.859,	0},
        {14.177, 4.191,	0},
        {13.455, 2.982,	-60},
        {13.796, 3.118,	-60}
    //    {12.25,2.95,60},
    //    {12.53,2.8,60},
    //    {11.73,4.17,0},
    //    {11.73,3.85,0},
    //    {12.55,5.27,-60},
    //    {12.28,5.11,-60},
    //    {13.89,5.09,-120},
    //    {13.63,5.28,-120},
    //    {14.38,4.19,180},
    //    {14.38,3.85,180},
    //    {13.58,2.8,120},
    //    {18.89,2.95,120}
    };

    private final SwerveSubsystem swerveSubsystem;
    private Translation2d[] reefSidePoints = new Translation2d[6];
    private Command[] reefPathCommands = new Command[12];
    private Pose2d[] reefPoses = new Pose2d[12];

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

    public PathRunner(SwerveSubsystem swerveSubsystem) {
        congifurePaths();
        this.swerveSubsystem = swerveSubsystem;
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

    public void GoToReef(int side, boolean isLeft) {
        currentCommand = reefPathCommands[side * 2 + (isLeft ? 1 : 0)];
        currentCommand.schedule();
    }

    public int getReefSide() {

        swerveSubsystem.getPose();
        int closestReefSide = 0;
        double closestDistance = 10000;
        for(int i = 0; i < 6; i++) { 
            double distance = swerveSubsystem.getPose().getTranslation().getDistance(reefSidePoints[i]);
            if(distance < closestDistance) {
                closestDistance = distance;
                closestReefSide = i;
            
            }
        }
        return closestReefSide;
    }

    private void congifurePaths() {

        DriverStation.getAlliance().ifPresent(
            alliance -> {
                isBlue = alliance == Alliance.Blue;
            }
        );

        double[] lastReefInfo = new double[2];
        int i = 0;
        for(double[] reefInfo : isBlue ? allBlueReefInfo : allRedReefInfo) {
            
            Pose2d reefPose = new Pose2d(reefInfo[0], reefInfo[1], Rotation2d.fromDegrees(reefInfo[2]));
            
            if(i % 2 == 1) {
                reefSidePoints[(i - 1)/2] = new Translation2d((reefInfo[0] + lastReefInfo[0])/2, (reefInfo[1] + lastReefInfo[1])/2);
            }

            reefPoses[i] = reefPose;

            reefPathCommands[i] = AutoBuilder.pathfindToPose(
                reefPose, 
                constraints,
                0.0
            );
            lastReefInfo = reefInfo;
            i++;
        }


        constraints = new PathConstraints(
            3.0, 3.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720)
        );

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
            boolean lTrigger = controller.getLeftTriggerAxis() > 0.1;
            boolean rTrigger = controller.getRightTriggerAxis() > 0.1;

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
                } else if (lTrigger) { 
                    GoToReef(getReefSide(), true);
                    commandRunning = true;
                } else if (rTrigger) {
                    GoToReef(getReefSide(), false);
                    commandRunning = true;

                } 
            }
        });     
    }
}
