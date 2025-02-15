package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.Percent;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
            .withDeadband(TunerConstants.MaxSpeed * 0.02).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.02)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentric driveDetailed = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
    //         .withDeadband(TunerConstants.MaxSpeed * 0.02).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.02)
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Test Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
        drivetrain.seedFieldCentric();

        NamedCommands.registerCommand("Align To Tag", null);
        NamedCommands.registerCommand("Score In Trough", troughScoreCommand());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * TunerConstants.MaxSpeed / 3)
                        .withVelocityY(-joystick.getLeftX() * TunerConstants.MaxSpeed / 3)
                        .withRotationalRate(-joystick.getRightX() * TunerConstants.MaxAngularRate)));

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
                        // () -> sQuickDraw.setVelocity(joystick2.getRightY() / 3), sQuickDraw));
                        () -> sQuickDraw.setSpeedFromElevatorPosition(sElevator.getPosition()), sQuickDraw));

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
                                joystick2.leftBumper().getAsBoolean(),
                                joystick2.rightBumper().getAsBoolean(),
                                joystick2.getRightY()),
                        sAlgaer));

        sLED.setDefaultCommand(
                new RunCommand(
                        () -> sLED.runPattern(
                                LEDPattern.solid(Color.kBlue).atBrightness(Percent.of(10))),
                        sLED));

        // sGetToDashboard.setDefaultCommand(new RunCommand(() ->
        // sGetToDashboard.getValues(
        // joystick2.button(1).getAsBoolean(),
        // joystick2.button(2).getAsBoolean(),
        // joystick2.button(3).getAsBoolean(),
        // joystick2.button(4).getAsBoolean(),
        // joystick2.button(5).getAsBoolean(),
        // joystick2.button(6).getAsBoolean(),
        // joystick2.button(7).getAsBoolean(),
        // joystick2.button(8).getAsBoolean(),
        // joystick2.button(9).getAsBoolean(),
        // joystick2.button(10).getAsBoolean(),
        // joystick2.button(11).getAsBoolean(),
        // joystick2.button(12).getAsBoolean(),
        // joystick2.button(13).getAsBoolean(),
        // joystick2.button(14).getAsBoolean(),
        // joystick2.button(15).getAsBoolean(),
        // joystick2.button(16).getAsBoolean(),
        // joystick2.button(17).getAsBoolean(),
        // joystick2.button(18).getAsBoolean(),
        // joystick2.button(19).getAsBoolean(),
        // joystick2.button(20).getAsBoolean()), sGetToDashboard));

        joystick.leftBumper().whileTrue(drivetrain.applyRequest(() -> driveDetailed
                .withVelocityX(drivetrain.getPoseForAlign().getX() * TunerConstants.MaxSpeed)
                .withVelocityY(drivetrain.getPoseForAlign().getY() * TunerConstants.MaxSpeed)
                .withRotationalRate(
                        drivetrain.getPoseForAlign().getRotation().getRadians() * TunerConstants.MaxAngularRate)));

        // joystick.rightBumper().whileTrue(drivetrain.applyRequest(() ->
        // driveRobotCentric
        // .withVelocityY(-0.05 * TunerConstants.MaxSpeed)));

        joystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick2.back().onTrue(
                new SequentialCommandGroup(
                        new RunCommand(() -> sIndexer.setFingerAndCannon(true, false, 0.0, 0.05), sIndexer)
                                .until(sIndexer.getTopFingerSwitch())
                                .withTimeout(0.5),
                        new WaitCommand(0.25),
                        new ParallelCommandGroup(
                                new RunCommand(
                                        () -> sIndexer.setFingerAndCannon(true, false, 0.5, 0), sIndexer)
                                        .withTimeout(0.4),
                                new SequentialCommandGroup(
                                        // new InstantCommand(() -> sIndexer.setFingerAndCannon(false, false, 0.5, 0.0),
                                        //         sIndexer),
                                        new InstantCommand(() -> sSpaceGun.shootMotor(-0.5), sSpaceGun),
                                        new WaitCommand(0.5),
                                        new InstantCommand(() -> sSpaceGun.shootMotor(0))))
                        .withTimeout(0.6),
                        new RunCommand(() -> sIndexer.setFingerAndCannon(false, true, 0.0, 0.0),
                                sIndexer)
                                .until(sIndexer.getBottomFingerSwitch())));

        joystick.rightBumper().onTrue(
                new SequentialCommandGroup(
                        new InstantCommand(() -> sSpaceGun.shootMotor(0.5), sSpaceGun),
                        new WaitCommand(0.375),
                        new InstantCommand(() -> sSpaceGun.shootMotor(0), sSpaceGun),
                        new ParallelCommandGroup(
                                new RunCommand(() -> sQuickDraw.setSpeedFromElevatorPosition(0), sQuickDraw),
                                new SequentialCommandGroup(
                                        new WaitCommand(0.25),
                                        new RunCommand(() -> sElevator.moveToSetpoint(false, true), sElevator)))
                                .until(sElevator.getBottomLimitSwitch())));

        joystick.b().onTrue(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new RunCommand(() -> sQuickDraw.setSpeedFromElevatorPosition(-1), sQuickDraw)
                                                .withTimeout(0.5)),
                                new SequentialCommandGroup(
                                        new WaitCommand(0.25),
                                        new InstantCommand(() -> sSpaceGun.shootMotor(0.5), sSpaceGun),
                                        new WaitCommand(0.25),
                                        new InstantCommand(() -> sSpaceGun.shootMotor(0), sSpaceGun)))));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Command troughScoreCommand() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new SequentialCommandGroup(
                            new RunCommand(() -> sQuickDraw.setSpeedFromElevatorPosition(-1), sQuickDraw)
                                    .withTimeout(0.5)),
                    new SequentialCommandGroup(
                            new WaitCommand(0.25),
                            new InstantCommand(() -> sSpaceGun.shootMotor(0.5), sSpaceGun),
                            new WaitCommand(0.25),
                            new InstantCommand(() -> sSpaceGun.shootMotor(0), sSpaceGun))));
    }
}
