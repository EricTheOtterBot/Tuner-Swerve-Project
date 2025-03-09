package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.Extras.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.Extras.LimelightHelpers;
import frc.robot.Extras.TunerConstants;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    private int lastTag = 1;

    private Pigeon2 gyro;

    private SwerveDrivePoseEstimator poseEstimator;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);

        gyro = new Pigeon2(TunerConstants.kPigeonId);

        poseEstimator = new SwerveDrivePoseEstimator(
                getKinematics(),
                gyro.getRotation2d(),
                getState().ModulePositions,
                new Pose2d(
                        0.0,
                        0.0,
                        new Rotation2d(
                                0.0)), 
                VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(2)), 
                VecBuilder.fill(2, 2, Units.degreesToRadians(10)));

        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getPose(), // Supplier of current robot pose Used to be getState().Pose but that wasn't
                                     // resetting to .5's //getPose()
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(0.5, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(1, 0, 0)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
            );

        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }

    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getSpeeds() {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }

    private Pose3d getTagPose3d(int tagNumber) {

        if (tagNumber >= 1 && tagNumber <= 22) {
            lastTag = tagNumber;
            return AprilTagFieldLayout.loadField(edu.wpi.first.apriltag.AprilTagFields.k2025Reefscape)
                    .getTagPose(tagNumber).get();
        } else {
            return AprilTagFieldLayout.loadField(edu.wpi.first.apriltag.AprilTagFields.k2025Reefscape)
                    .getTagPose(lastTag).get();
        }
    }

    private boolean isTarget() {
        if (LimelightHelpers.getFiducialID("limelight-otto") != -1) {
            return true;
        } else {
            return false;
        }
    }


    private int getLastTag() {
        int tag = (int) LimelightHelpers.getFiducialID("limelight-otto");
        if (tag >= 1 && tag <= 22) {
            lastTag = tag;
            return tag;
        } else {
            return lastTag;
        }
    }

    public Pose2d robotPoseGivenTagPoseRight() {
        Pose3d pose = getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto"));
        return new Pose2d(
            pose.getX() 
        + pose.getRotation().toRotation2d().getCos() * 0.8
        + Math.cos(pose.getRotation().toRotation2d().getRadians() + Math.PI / 2) * 0.330,

        pose.getY() 
        + pose.getRotation().toRotation2d().getSin() * 0.8
        + Math.sin(pose.getRotation().toRotation2d().getRadians() + Math.PI / 2) * 0.330,

        Rotation2d.fromRadians(pose.getRotation().toRotation2d().getRadians() + Math.PI));
    }

    public Pose2d robotPoseGivenTagPoseLeft() {
        Pose3d pose = getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto"));
        return new Pose2d(
            pose.getX() 
        + pose.getRotation().toRotation2d().getCos() * 0.7
        + Math.cos(pose.getRotation().toRotation2d().getRadians() + Math.PI / 2) * 0.05,

        pose.getY() 
        + pose.getRotation().toRotation2d().getSin() * 0.7
        + Math.sin(pose.getRotation().toRotation2d().getRadians() + Math.PI / 2) * 0.05,

        Rotation2d.fromRadians(pose.getRotation().toRotation2d().getRadians() + Math.PI));
    }

    public Command getAlignRightReef(Pose2d startPose) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                startPose,
                //new Pose2d(startPose.getX(), startPose.getY(), robotPoseGivenTagPoseRight().getRotation()),
                robotPoseGivenTagPoseRight());
        // The values are low so if anything goes wrong we can disable the robot
        PathConstraints constraints = new PathConstraints(1.5, 1.0, 2 * Math.PI, 4 * Math.PI);
        PathPlannerPath alignmentPath = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0, Rotation2d.fromRadians(getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getRotation().toRotation2d().getRadians() + Math.PI)));

                alignmentPath.preventFlipping = true;
        return AutoBuilder.followPath(alignmentPath);
        // return AutoBuilder.pathfindThenFollowPath(alignmentPath, constraints);
    }

    public Command getAlignLeftReef(Pose2d startPose) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                startPose,
                //new Pose2d(startPose.getX(), startPose.getY(), robotPoseGivenTagPoseLeft().getRotation()),
                robotPoseGivenTagPoseLeft());
        // The values are low so if anything goes wrong we can disable the robot
        PathConstraints constraints = new PathConstraints(2.0, 1.5, 2 * Math.PI, 4 * Math.PI);
        PathPlannerPath alignmentPath = new PathPlannerPath(
                waypoints,
                constraints,
                new IdealStartingState(pythagorean(getSpeeds().vxMetersPerSecond, getSpeeds().vyMetersPerSecond), getState().RawHeading),
                new GoalEndState(0.2, Rotation2d.fromRadians(getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getRotation().toRotation2d().getRadians() + Math.PI)));

                alignmentPath.preventFlipping = true;

        return AutoBuilder.followPath(alignmentPath);
        // return AutoBuilder.pathfindThenFollowPath(alignmentPath, constraints);
    }

    public double pythagorean(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    } 

    @Override
    public void periodic() {

        poseEstimator.update(gyro.getRotation2d(), getState().ModulePositions);

        if (isTarget()) {
            poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiBlue("limelight-otto"),
                    Timer.getFPGATimestamp());
        }

        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        SmartDashboard.putNumber("Limelight Pose X", LimelightHelpers.getBotPose3d_wpiBlue("limelight-otto").getX());
        SmartDashboard.putNumber("Limelight Pose Y", LimelightHelpers.getBotPose3d_wpiBlue("limelight-otto").getY());

        SmartDashboard.putNumber("Limelight Tag T X",
                getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getX());
        SmartDashboard.putNumber("Limelight Tag T Y",
                getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getY());
        SmartDashboard.putNumber("Limelight Tag T Z",
                getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getZ());
        SmartDashboard.putNumber("Limelight Tag R X",
                getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getRotation().getX());
        SmartDashboard.putNumber("Limelight Tag R Y",
                getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getRotation().getY());
        SmartDashboard.putNumber("Limelight Tag R Z",
                getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getRotation().getAngle());

        SmartDashboard.putNumber("PE Pose X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("PE Pose Y", poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("PE Pose Rotation", poseEstimator.getEstimatedPosition().getRotation().getRadians());

        SmartDashboard.putNumber("Tag #", getLastTag());

        SmartDashboard.putNumber("New Test Pose X", robotPoseGivenTagPoseRight().getX());
        SmartDashboard.putNumber("New Test Pose Y", robotPoseGivenTagPoseRight().getY());
        SmartDashboard.putNumber("New Test Pose Radians", robotPoseGivenTagPoseRight().getRotation().getRadians());

        SmartDashboard.putNumber("CTRE Pose X", getState().Pose.getX());
        SmartDashboard.putNumber("CTRE Pose Y", getState().Pose.getY());
        SmartDashboard.putNumber("CTRE Pose Rotation", getState().Pose.getRotation().getRadians());


    }
}
