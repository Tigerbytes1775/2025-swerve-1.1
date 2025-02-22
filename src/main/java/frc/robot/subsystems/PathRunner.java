package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import swervelib.SwerveDrive;

public class PathRunner extends SubsystemBase {

    private final Pose2d feederPose;

    private final PathConstraints constraints;

    private final Command goToFeeder;
    private Command currentCommand;

    public PathRunner() {
        this.feederPose = new Pose2d(1.177, 6.957, Rotation2d.fromDegrees(-126.076));
        this.constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        goToFeeder = AutoBuilder.pathfindToPose(
            feederPose, 
            constraints, 
            0.0
        );
    }

    

    public void stopPaths() {
        currentCommand.cancel();
    }

    public void goToFeeder() {
        currentCommand = goToFeeder;
        currentCommand.schedule();
    }
}
