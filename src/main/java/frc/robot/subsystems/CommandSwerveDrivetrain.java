package frc.robot.subsystems;

import java.util.List;
import java.util.function.BooleanSupplier;
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
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.Extras.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.Extras.AprilTagLocations;
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

    private PIDController controllerX;
    private PIDController controllerY;
    private PIDController controllerRotation;

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
                                0.0)));
        controllerX = new PIDController(0.7, 0.0, 0.0);
        controllerY = new PIDController(0.7, 0.0, 0.0);
        controllerRotation = new PIDController(2.0, 0.0, 0.0);
        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(10, 0, 0),
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

    public Pose2d getPoseForAlign() {
        Pose2d desiredPose;
        int tag = getLastTag();
        if (tag == 17) {
            desiredPose = AprilTagLocations.tag17;
        } else if (tag == 18) {
            desiredPose = AprilTagLocations.tag18;
        } else if (tag == 19) {
            desiredPose = AprilTagLocations.tag19;
        } else if (tag == 20) {
            desiredPose = AprilTagLocations.tag20;
        } else if (tag == 21) {
            desiredPose = AprilTagLocations.tag21;
        } else if (tag == 22) {
            desiredPose = AprilTagLocations.tag22;
        } else {
            desiredPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        }
        Pose2d currentPose = poseEstimator.getEstimatedPosition();
        double x = controllerX.calculate(currentPose.getX(), desiredPose.getX());
        double y = controllerY.calculate(currentPose.getY(), desiredPose.getY());
        double rotation = controllerRotation.calculate(currentPose.getRotation().getRadians(),
        desiredPose.getRotation().getRadians());
        if(x > 0.2) {
            x = 0.2;
        } else if(x < -0.2) {
            x = -0.2;
        }
        if(y > 0.2) {
            y = 0.2;
        } else if(y < -0.2) {
            y = -0.2;
        }
        if(rotation > 0.2) {
            rotation = 0.2; 
        } else if(rotation < -0.2) {
            rotation = -0.2;
        }
        Pose2d poseOffset = new Pose2d(x, y, new Rotation2d(rotation));
        return poseOffset;
    }

    public void resetPosePose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    public BooleanSupplier isVisionTracked() {
        if(getPoseForAlign().getX() + getPoseForAlign().getY() + getPoseForAlign().getRotation().getRadians() < 0.05) {
            return () -> true;
        } else {
            return () -> false;
        }
    }

    public Pose2d getTagPoseForAlign() {
        Pose2d desiredPose;
        int tag = getLastTag();
        if (tag == 17) {
            desiredPose = AprilTagLocations.tag17;
        } else if (tag == 18) {
            desiredPose = AprilTagLocations.tag18;
        } else if (tag == 19) {
            desiredPose = AprilTagLocations.tag19;
        } else if (tag == 20) {
            desiredPose = AprilTagLocations.tag20;
        } else if (tag == 21) {
            desiredPose = AprilTagLocations.tag21;
        } else if (tag == 22) {
            desiredPose = AprilTagLocations.tag22;
        } else {
            desiredPose = getPose();
        }
        return desiredPose;
    }

    public Command getAlignRightReef() {
        Pose2d curPose = poseEstimator.getEstimatedPosition();
        Pose2d goalPose = new Pose2d(5.35, 3.00, Rotation2d.fromRadians(2.33));
    
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(curPose.getX(), curPose.getY(), Rotation2d.fromDegrees(0)),
            new Pose2d(goalPose.getX(), goalPose.getY(), Rotation2d.fromDegrees(0))
        );
        // The values are low so if anything goes wrong we can disable the robot
        PathConstraints constraints = new PathConstraints(0.5, 0.5, 2 * Math.PI, 4 * Math.PI);
        PathPlannerPath alignmentPath = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0, goalPose.getRotation())
        );
    
        this.resetPose(poseEstimator.getEstimatedPosition());
        return AutoBuilder.followPath(alignmentPath);
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

        SmartDashboard.putNumber("Pose X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Pose Y", poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Pose Rotation", poseEstimator.getEstimatedPosition().getRotation().getRadians());

        SmartDashboard.putNumber("Tag #", getLastTag());

        SmartDashboard.putBoolean("Is Vision Aligned", isVisionTracked().getAsBoolean());

        SmartDashboard.putNumber("Test Tag X", getTagPoseForAlign().getX());
        SmartDashboard.putNumber("Test Tag Y", getTagPoseForAlign().getY());
        SmartDashboard.putNumber("Test Tag Rotation", getTagPoseForAlign().getRotation().getRadians());

    }
}
