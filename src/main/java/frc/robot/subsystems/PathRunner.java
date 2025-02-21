package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PathRunner extends SubsystemBase {



    Pose2d feederPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));

    PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    /*Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0// Goal end velocity in meters/sec

    );*/

//    public void stopPaths() {
//        pathPlannerFollower.cancelPath();
//    }

    public void goToFeeder() {
        AutoBuilder.pathfindToPose(feederPose, constraints, 0.0).schedule();
    }
}
