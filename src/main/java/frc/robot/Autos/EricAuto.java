package frc.robot.Autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class EricAuto {

    List<Waypoint> basicWaypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(0.0,0.0,Rotation2d.fromRadians(0)), 
        new Pose2d()
    );

    PathConstraints constraints = new PathConstraints(1.0, 0.5, 2 * Math.PI, 4 * Math.PI);

    public Command followPathCommand() {
        try {
            PathPlannerPath path = new PathPlannerPath(
                basicWaypoints, 
                constraints, 
                null, 
                new GoalEndState(0.0, Rotation2d.fromRadians(0.0))
            );

            return new RunCommand(() -> AutoBuilder.followPath(path));
        } catch(Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public Command followPathCommand(List<Waypoint> waypoints) {
        try {
            PathPlannerPath path = new PathPlannerPath(
                waypoints, 
                constraints, 
                null, 
                new GoalEndState(0.0, Rotation2d.fromRadians(0.0))
            );

            return new RunCommand(() -> AutoBuilder.followPath(path));
        } catch(Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
}
