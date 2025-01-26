package frc.robot.subsystems;


import java.util.function.Supplier;


import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
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
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private int lastTag = 1;

    private Pigeon2 gyro;

    private SwerveDrivePoseEstimator poseEstimator;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
    
    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
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
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);

        
    }

    private Pose3d getTagPose3d(int tagNumber) {
        
        if(tagNumber >= 1 && tagNumber <= 22) {
            lastTag = tagNumber;
            return AprilTagFieldLayout.loadField(edu.wpi.first.apriltag.AprilTagFields.k2025Reefscape).getTagPose(tagNumber).get();
        } else {
            return AprilTagFieldLayout.loadField(edu.wpi.first.apriltag.AprilTagFields.k2025Reefscape).getTagPose(lastTag).get();
        }
    }

    private boolean isTarget() {
        if(LimelightHelpers.getFiducialID("limelight-otto") != -1) {
            return true;
        } else {
            return false;
        }
    }

    public double alignToTagX() {
        double tagValue = getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getX();
        double tagRotation = getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getRotation().getAngle();
        double robotValue = poseEstimator.getEstimatedPosition().getX();
        return tagValue - robotValue;
    }

    public double alignToTagY() {
        double tagValue = getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getY();
        double tagRotation = getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getRotation().getAngle();
        double robotValue = poseEstimator.getEstimatedPosition().getY();
        return tagValue - robotValue;
    }

    public double alignToTagRotation() {
        double tagValue = getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getRotation().getAngle();
        if(tagValue < 0) {
            tagValue = tagValue + Math.PI;
        }
        double robotValue = poseEstimator.getEstimatedPosition().getRotation().getRadians();
        if(robotValue < 0) {
            robotValue = robotValue + Math.PI + Math.PI;
        }
        return tagValue - robotValue - Math.PI;
    }

    @Override
    public void periodic() {

        poseEstimator.update(gyro.getRotation2d(), getState().ModulePositions);

        if(isTarget()) {
            poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiBlue("limelight-otto"),  Timer.getFPGATimestamp());
        }


        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        SmartDashboard.putNumber("Limelight Pose X", LimelightHelpers.getBotPose3d_wpiBlue("limelight-otto").getX());
        SmartDashboard.putNumber("Limelight Pose Y", LimelightHelpers.getBotPose3d_wpiBlue("limelight-otto").getY());

        SmartDashboard.putNumber("Limelight Tag T X", getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getX());
        SmartDashboard.putNumber("Limelight Tag T Y", getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getY());
        SmartDashboard.putNumber("Limelight Tag T Z", getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getZ());
        SmartDashboard.putNumber("Limelight Tag R X", getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getRotation().getX());
        SmartDashboard.putNumber("Limelight Tag R Y", getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getRotation().getY());
        SmartDashboard.putNumber("Limelight Tag R Z", getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getRotation().getAngle());

        SmartDashboard.putNumber("Pose X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Pose Y", poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Pose Rotation", poseEstimator.getEstimatedPosition().getRotation().getRadians());

        SmartDashboard.putNumber("Subtracted X", alignToTagX());
        SmartDashboard.putNumber("Subtracted Y", alignToTagY());
        SmartDashboard.putNumber("Subtracted Rotation", alignToTagRotation());

    }
}
