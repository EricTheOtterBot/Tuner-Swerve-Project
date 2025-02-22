package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.Percent;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Extras.Telemetry;
import frc.robot.Extras.TunerConstants;
import frc.robot.subsystems.Algaer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
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

    public final CommandSwerveDrivetrain sDrive = TunerConstants.createDrivetrain();
    public final Elevator sElevator = new Elevator();
    public final LED sLED = new LED();
    public final SpaceGun sSpaceGun = new SpaceGun();
    public final QuickDraw sQuickDraw = new QuickDraw();
    public final Indexer sIndexer = new Indexer();
    public final Algaer sAlgaer = new Algaer();
    public final Climber sClimber = new Climber();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        NamedCommands.registerCommand("Tag Align X",
                new RunCommand(() -> PPHolonomicDriveController.overrideXFeedback(() -> {
                    return 0.2;
                })));// () -> drivetrain.getPoseForAlign().getX()

        NamedCommands.registerCommand("Tag Align Y",
                new RunCommand(() -> PPHolonomicDriveController.overrideYFeedback(() -> {
                    return 0.2;
                })));// () -> drivetrain.getPoseForAlign().getY()

        NamedCommands.registerCommand("Tag Align Rotation",
                new RunCommand(() -> PPHolonomicDriveController.overrideRotationFeedback(() -> {
                    return 0.2;
                })));// () -> drivetrain.getPoseForAlign().getRotation().getRadians()

        // acd
        NamedCommands.registerCommand("Clear PP Overrides",
                new InstantCommand(() -> PPHolonomicDriveController.clearFeedbackOverrides()));

        NamedCommands.registerCommand("Score In Trough", troughScoreCommand());
        NamedCommands.registerCommand("Put Coral On SpaceGun", putCoralOnSpaceGunCommand());
        NamedCommands.registerCommand("Reset Gyro 180", zeroGyro180OffCommand());
        NamedCommands.registerCommand("Go Right", sDrive.applyRequest(
                () -> driveRobotCentric.withVelocityY(-0.05 * TunerConstants.MaxSpeed).withVelocityX(0.0)));

        NamedCommands.registerCommand("aaron", new RunCommand(() -> {
            Pose2d startPos = sDrive.getPose();
            Pose2d endPos = sDrive.getTagPoseForAlign();//new Pose2d(0.2,0.2, Rotation2d.fromRadians(0.0));
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
            PathPlannerPath path = new PathPlannerPath(waypoints,
                    new PathConstraints(4.21, 0.5, 9.42, 12.56),
                    new IdealStartingState(0, new Rotation2d(0.0)),
                    new GoalEndState(0.0, new Rotation2d(0.0)));
            path.preventFlipping = true;
            AutoBuilder.followPath(path).schedule();
        }));

        autoChooser = AutoBuilder.buildAutoChooser("Test Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);
        autoChooser.addOption("Combo Auto", comboAutoCommand());

        configureDefaultBindings();
        configureBindings();
        sDrive.seedFieldCentric();
    }

    private void configureDefaultBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        sDrive.setDefaultCommand(
                // Drivetrain will execute this command periodically
                sDrive.applyRequest(() -> drive
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

        // sLED.setDefaultCommand(
        //         new RunCommand(
        //                 () -> sLED.runPattern(
        //                         LEDPattern.solid(DriverStation.getAlliance().ifPresent(

        //                                     DriverStation.getAlliance().toString() == "Red"
        //                                             ? Color.kRed
        //                                             : Color.kBlue
        //                         ))
        //                                 .atBrightness(Percent.of(10))),
        //                 sLED));

        sClimber.setDefaultCommand(
                new RunCommand(
                        () -> sClimber.setMotor(
                                joystick2.leftBumper().getAsBoolean(),
                                joystick2.rightBumper().getAsBoolean()),
                        sClimber));
    }

    private void configureBindings() {
        joystick.leftBumper().whileTrue(sDrive.applyRequest(() -> driveDetailed
                .withVelocityX(sDrive.getPoseForAlign().getX() * TunerConstants.MaxSpeed)
                .withVelocityY(sDrive.getPoseForAlign().getY() * TunerConstants.MaxSpeed)
                .withRotationalRate(
                        sDrive.getPoseForAlign().getRotation().getRadians()
                                * TunerConstants.MaxAngularRate)));
        joystick.povRight().whileTrue(sDrive.applyRequest(
                () -> driveRobotCentric.withVelocityY(-0.05 * TunerConstants.MaxSpeed).withVelocityX(0.0)));
        joystick.povLeft().whileTrue(sDrive.applyRequest(
                () -> driveRobotCentric.withVelocityY(0.05 * TunerConstants.MaxSpeed).withVelocityX(0.0)));
        joystick.povUp().whileTrue(sDrive.applyRequest(
                () -> driveRobotCentric.withVelocityX(0.05 * TunerConstants.MaxSpeed).withVelocityY(0.0)));
        joystick.povDown().whileTrue(sDrive.applyRequest(
                () -> driveRobotCentric.withVelocityX(-0.05 * TunerConstants.MaxSpeed).withVelocityY(0.0)));
        joystick.povUpLeft().whileTrue(sDrive.applyRequest(() -> driveRobotCentric
                .withVelocityX(0.05 * TunerConstants.MaxSpeed).withVelocityY(0.05 * TunerConstants.MaxSpeed)));
        joystick.povUpRight().whileTrue(sDrive.applyRequest(() -> driveRobotCentric
                .withVelocityX(0.05 * TunerConstants.MaxSpeed).withVelocityY(-0.05 * TunerConstants.MaxSpeed)));
        joystick.povDownLeft().whileTrue(sDrive.applyRequest(() -> driveRobotCentric
                .withVelocityX(-0.05 * TunerConstants.MaxSpeed).withVelocityY(0.05 * TunerConstants.MaxSpeed)));
        joystick.povDownRight().whileTrue(sDrive.applyRequest(() -> driveRobotCentric
                .withVelocityX(-0.05 * TunerConstants.MaxSpeed).withVelocityY(-0.05 * TunerConstants.MaxSpeed)));
        joystick.x().onTrue(sDrive.runOnce(() -> sDrive.seedFieldCentric()));
        joystick.rightBumper().onTrue(pipeScoreCommand());
        joystick.b().onTrue(troughScoreCommand());
        joystick.a().whileTrue(sDrive.getAlignRightReef());

        joystick.start().onTrue(new InstantCommand(() -> sDrive.resetPosePose(new Pose2d(0.5,0.5,Rotation2d.fromRadians(0.0)))));

        joystick2.back().onTrue(putCoralOnSpaceGunCommand());

        sClimber.getSwitch().whileTrue(new RunCommand(() -> sLED.runPattern(LEDPattern.solid(Color.kGreen)
                .atBrightness(Percent.of(25)))));

        sDrive.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Command ericCommand() {
        
        try{
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile("Test Path");
    
            // Create a path following command using AutoBuilder. This will also trigger event markers.
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
      }

      public Command edawgCommand() {
        return sDrive.applyRequest(() -> driveDetailed
        .withVelocityX(sDrive.getPoseForAlign().getX() * TunerConstants.MaxSpeed)
        .withVelocityY(sDrive.getPoseForAlign().getY() * TunerConstants.MaxSpeed)
        .withRotationalRate(
                sDrive.getPoseForAlign().getRotation().getRadians()
                        * TunerConstants.MaxAngularRate));
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
                                new InstantCommand(() -> sSpaceGun.shootMotor(0.3),
                                        sSpaceGun),
                                new WaitCommand(0.6),
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
        return sDrive.applyRequest(() -> driveDetailed
                .withVelocityX(sDrive.getPoseForAlign().getX() * TunerConstants.MaxSpeed)
                .withVelocityY(sDrive.getPoseForAlign().getY() * TunerConstants.MaxSpeed)
                .withRotationalRate(
                        sDrive.getPoseForAlign().getRotation().getRadians()
                                * TunerConstants.MaxAngularRate))
                .until(sDrive.isVisionTracked());
    }

    public Command zeroGyro180OffCommand() {
        return new InstantCommand(() -> sDrive.resetPose(new Pose2d(0.0, 0.0, new Rotation2d(-3.1))), sDrive);
    }

    public Command comboAutoCommand() {
        return new SequentialCommandGroup(
            //new InstantCommand(() -> sDrive.resetPosePose(new Pose2d(0.5,0.5,Rotation2d.fromRadians(0.0)))),
            //new WaitCommand(0.25),
            edawgCommand()
            // new RunCommand({
            //     try {
            //         PathPlannerPath path = PathPlannerPath.fromPathFile("R-Start to BR-Reef");
            //         return AutoBuilder.followPath(path);
            //     } catch (FileVersionException e) {
            //         // TODO Auto-generated catch block
            //         e.printStackTrace();
            //         return Commands.none();
            //     } catch (IOException e) {
            //         // TODO Auto-generated catch block
            //         e.printStackTrace();
            //         return Commands.none();
            //     } catch (ParseException e) {
            //         // TODO Auto-generated catch block
            //         e.printStackTrace();
            //         return Commands.none();
            //     }
            //  } )
            
            // new RunCommand(() -> {
             
            //         try {
            //             AutoBuilder.followPath(PathPlannerPath.fromPathFile("R-Start to BR-Reef")).schedule();
            //         } catch (FileVersionException e) {
            //             // TODO Auto-generated catch block
            //             e.printStackTrace();
            //         } catch (IOException e) {
            //             // TODO Auto-generated catch block
            //             e.printStackTrace();
            //         } catch (ParseException e) {
            //             // TODO Auto-generated catch block
            //             e.printStackTrace();
            //         }
              
            // })
                //sDrive.applyRequest(() -> driveDetailed
                //.withVelocityX(sDrive.getPoseForAlign().getX() * TunerConstants.MaxSpeed)
                //.withVelocityY(sDrive.getPoseForAlign().getY() * TunerConstants.MaxSpeed)
                //.withRotationalRate(
                //        sDrive.getPoseForAlign().getRotation().getRadians()
                //                * TunerConstants.MaxAngularRate))
        );
    }
}
