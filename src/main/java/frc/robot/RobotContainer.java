package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.Percent;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Extras.Telemetry;
import frc.robot.Extras.TunerConstants;
import frc.robot.subsystems.Algaer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GetToDashboard;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.QuickDraw;
import frc.robot.subsystems.SpaceGun;

public class RobotContainer {
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.MaxSpeed * 0.02)
            .withRotationalDeadband(TunerConstants.MaxAngularRate * 0.02)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentric driveDetailed = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(TunerConstants.MaxSpeed *
                    0.02)
            .withRotationalDeadband(TunerConstants.MaxAngularRate * 0.02)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(TunerConstants.MaxSpeed);

    public final CommandXboxController joystick = new CommandXboxController(0);
    public final CommandXboxController joystick2 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator sElevator = new Elevator();
    public final LED sLED = new LED();
    public final SpaceGun sSpaceGun = new SpaceGun();
    public final QuickDraw sQuickDraw = new QuickDraw();
    public final Indexer sIndexer = new Indexer();
    public final Algaer sAlgaer = new Algaer();
    public final GetToDashboard sGetToDashboard = new GetToDashboard();
    public final Climber sClimber = new Climber();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Align To Tag", alignToTagCommand());

        // acd- Does this need to be an InstantCommand?
        NamedCommands.registerCommand("Align To Tag 2", new InstantCommand(
                () -> PPHolonomicDriveController.overrideXYFeedback(() -> {
                    return 0.5;
                }, () -> {
                    return 0.5;
                })));

        NamedCommands.registerCommand("Tag Align X",
                new RunCommand(() -> PPHolonomicDriveController.overrideXFeedback(() -> {return 0.2;})));//() -> drivetrain.getPoseForAlign().getX()

        NamedCommands.registerCommand("Tag Align Y",
                new RunCommand(() -> PPHolonomicDriveController.overrideYFeedback(() -> {return 0.2;})));//() -> drivetrain.getPoseForAlign().getY()

        NamedCommands.registerCommand("Tag Align Rotation",
                new RunCommand(() -> PPHolonomicDriveController.overrideRotationFeedback(() -> {return 0.2;})));//() -> drivetrain.getPoseForAlign().getRotation().getRadians()
        
        // acd
        NamedCommands.registerCommand("Clear PP Overrides",
                new InstantCommand(() -> PPHolonomicDriveController.clearFeedbackOverrides()));

        NamedCommands.registerCommand("Score In Trough", troughScoreCommand());
        NamedCommands.registerCommand("Put Coral On SpaceGun", putCoralOnSpaceGunCommand());
        NamedCommands.registerCommand("Reset Gyro 180", zeroGyro180OffCommand());
        NamedCommands.registerCommand("Go Right", drivetrain.applyRequest(
                () -> driveRobotCentric.withVelocityY(-0.05 * TunerConstants.MaxSpeed).withVelocityX(0.0)));

        NamedCommands.registerCommand("aaron", new RunCommand(() -> {
            Pose2d currentPose = drivetrain.getPose();
            Pose2d startPos = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());
            Pose2d endPos = drivetrain.getTagPoseForAlign();
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
            PathPlannerPath path = new PathPlannerPath(waypoints, 
                new PathConstraints(2.0, 0.25, 9.42, 4*3.142), 
                null, // new IdealStartingState(0, new Rotation2d(1.23)),
                new GoalEndState(0.0, new Rotation2d(2.33)));
            path.preventFlipping = true;
            AutoBuilder.followPath(path).schedule();
        }));

        autoChooser = AutoBuilder.buildAutoChooser("Test Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);
        

        configureBindings();
        drivetrain.seedFieldCentric();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-joystick.getLeftY() * TunerConstants.MaxSpeed / 3)
                        .withVelocityY(-joystick.getLeftX() * TunerConstants.MaxSpeed / 3)
                        .withRotationalRate(-joystick.getRightX()
                                * TunerConstants.MaxAngularRate)));

        sElevator.setDefaultCommand(
                new RunCommand(
                        () -> sElevator.moveToSetpoint(
                                joystick2.y().getAsBoolean(),
                                joystick2.a().getAsBoolean()),
                        sElevator));

        sSpaceGun.setDefaultCommand(
                new RunCommand(
                        () -> sSpaceGun.shootMotor(joystick2.getLeftY() / 1.5), sSpaceGun));

        sQuickDraw.setDefaultCommand(
                new RunCommand(
                        () -> sQuickDraw.setVelocity(joystick2.getRightY() / 3),
                        sQuickDraw));
        // () -> sQuickDraw.setSpeedFromElevatorPosition(sElevator.getPosition()),
        // sQuickDraw));

        sIndexer.setDefaultCommand(
                new RunCommand(() -> sIndexer.setFingerAndCannon(
                        joystick2.b().getAsBoolean(),
                        joystick2.x().getAsBoolean(),
                        joystick2.getLeftTriggerAxis(),
                        joystick2.getRightTriggerAxis()),
                        sIndexer));

        sAlgaer.setDefaultCommand(
                new RunCommand(
                        () -> sAlgaer.setMotors(
                                joystick2.povLeft().getAsBoolean(),
                                joystick2.povRight().getAsBoolean(),
                                joystick2.povDown().getAsBoolean(),
                                joystick2.povUp().getAsBoolean()),
                        sAlgaer));

        sLED.setDefaultCommand(
                new RunCommand(
                        () -> sLED.runPattern(
                                LEDPattern.solid(Color.kBlue)
                                        .atBrightness(Percent.of(10))),
                        sLED));

        sClimber.setDefaultCommand(
                new RunCommand(
                        () -> sClimber.setMotor(
                                joystick2.leftBumper().getAsBoolean(),
                                joystick2.rightBumper().getAsBoolean()),
                        sClimber));

        joystick.leftBumper().whileTrue(drivetrain.applyRequest(() -> driveDetailed
                .withVelocityX(drivetrain.getPoseForAlign().getX() * TunerConstants.MaxSpeed)
                .withVelocityY(drivetrain.getPoseForAlign().getY() * TunerConstants.MaxSpeed)
                .withRotationalRate(
                        drivetrain.getPoseForAlign().getRotation().getRadians()
                                * TunerConstants.MaxAngularRate)));

        joystick.povRight().whileTrue(drivetrain.applyRequest(
                () -> driveRobotCentric.withVelocityY(-0.05 * TunerConstants.MaxSpeed).withVelocityX(0.0)));
        joystick.povLeft().whileTrue(drivetrain.applyRequest(
                () -> driveRobotCentric.withVelocityY(0.05 * TunerConstants.MaxSpeed).withVelocityX(0.0)));
        joystick.povUp().whileTrue(drivetrain.applyRequest(
                () -> driveRobotCentric.withVelocityX(0.05 * TunerConstants.MaxSpeed).withVelocityY(0.0)));
        joystick.povDown().whileTrue(drivetrain.applyRequest(
                () -> driveRobotCentric.withVelocityX(-0.05 * TunerConstants.MaxSpeed).withVelocityY(0.0)));
        joystick.povUpLeft().whileTrue(drivetrain.applyRequest(() -> driveRobotCentric
                .withVelocityX(0.05 * TunerConstants.MaxSpeed).withVelocityY(0.05 * TunerConstants.MaxSpeed)));
        joystick.povUpRight().whileTrue(drivetrain.applyRequest(() -> driveRobotCentric
                .withVelocityX(0.05 * TunerConstants.MaxSpeed).withVelocityY(-0.05 * TunerConstants.MaxSpeed)));
        joystick.povDownLeft().whileTrue(drivetrain.applyRequest(() -> driveRobotCentric
                .withVelocityX(-0.05 * TunerConstants.MaxSpeed).withVelocityY(0.05 * TunerConstants.MaxSpeed)));
        joystick.povDownRight().whileTrue(drivetrain.applyRequest(() -> driveRobotCentric
                .withVelocityX(-0.05 * TunerConstants.MaxSpeed).withVelocityY(-0.05 * TunerConstants.MaxSpeed)));

        joystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick2.back().onTrue(putCoralOnSpaceGunCommand());

        joystick.rightBumper().onTrue(pipeScoreCommand());

        joystick.b().onTrue(troughScoreCommand());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Command troughScoreCommand() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new RunCommand(() -> sQuickDraw
                                        .setSpeedFromElevatorPosition(-1),
                                        sQuickDraw)
                                        .withTimeout(0.5)),
                        new SequentialCommandGroup(
                                new WaitCommand(0.25),
                                new InstantCommand(() -> sSpaceGun.shootMotor(0.5),
                                        sSpaceGun),
                                new WaitCommand(0.4),
                                new InstantCommand(() -> sSpaceGun.shootMotor(0),
                                        sSpaceGun))),
                new RunCommand(() -> sQuickDraw.setSpeedFromElevatorPosition(0), sQuickDraw)
                        .withTimeout(0.75));
        // .withTimeout(2);
    }

    public Command putCoralOnSpaceGunCommand() {
        return new SequentialCommandGroup(
                new RunCommand(() -> sIndexer.setFingerAndCannon(true, false, 0.0,
                        0.05), sIndexer)
                        .withTimeout(0.5),
                // new WaitCommand(0.5),
                new ParallelCommandGroup(
                        new RunCommand(
                                () -> sIndexer.setFingerAndCannon(true,
                                        false, 0.5, 0),
                                sIndexer)
                                .withTimeout(0.4),
                        new SequentialCommandGroup(
                                // new InstantCommand(() ->
                                // sIndexer.setFingerAndCannon(false,
                                // false, 0.5, 0.0),
                                // sIndexer),
                                new InstantCommand(() -> sSpaceGun
                                        .shootMotor(-0.5),
                                        sSpaceGun),
                                new WaitCommand(0.5),
                                new InstantCommand(() -> sSpaceGun
                                        .shootMotor(0))))
                        .withTimeout(0.6),
                new RunCommand(() -> sIndexer.setFingerAndCannon(false, true, 0.0, 0.0),
                        sIndexer)
                        .withTimeout(0.5));
    }

    public Command pipeScoreCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> sSpaceGun.shootMotor(0.4), sSpaceGun),
                new WaitCommand(0.5),
                new InstantCommand(() -> sSpaceGun.shootMotor(0), sSpaceGun),
                new ParallelCommandGroup(
                        new RunCommand(() -> sQuickDraw
                                .setSpeedFromElevatorPosition(0),
                                sQuickDraw),
                        new SequentialCommandGroup(
                                new WaitCommand(0.25),
                                new RunCommand(() -> sElevator
                                        .moveToSetpoint(false,
                                                true),
                                        sElevator)))
                        .until(sElevator.getBottomLimitSwitch()));
    }

    public Command alignToTagCommand() {
        return drivetrain.applyRequest(() -> driveDetailed
                .withVelocityX(drivetrain.getPoseForAlign().getX() * TunerConstants.MaxSpeed)
                .withVelocityY(drivetrain.getPoseForAlign().getY() * TunerConstants.MaxSpeed)
                .withRotationalRate(
                        drivetrain.getPoseForAlign().getRotation().getRadians()
                                * TunerConstants.MaxAngularRate))
                .until(drivetrain.isVisionTracked());
    }

    public Command zeroGyro180OffCommand() {
        return new InstantCommand(() -> drivetrain.resetPose(new Pose2d(0.0, 0.0, new Rotation2d(-3.1))), drivetrain);
    }

}
