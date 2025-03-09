package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import java.util.Set;

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

    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(TunerConstants.MaxSpeed *
                    0.02)
            .withRotationalDeadband(TunerConstants.MaxAngularRate * 0.02)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(TunerConstants.MaxSpeed);

    public final CommandXboxController joystick = new CommandXboxController(1);
    public final CommandXboxController joystick2 = new CommandXboxController(0);

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

        NamedCommands.registerCommand("Score In Trough", troughScoreCommand());
        NamedCommands.registerCommand("Put Coral On SpaceGun", putCoralOnSpaceGunCommand());
        NamedCommands.registerCommand("Pipe Score", pipeScoreCommand().withTimeout(3));
        NamedCommands.registerCommand("Increment Counter", new InstantCommand(() -> sElevator.incrementCounter(), sElevator));
        NamedCommands.registerCommand("Move Elevator", new RunCommand(() -> sElevator.moveToSetpointWithCounter(), sElevator).withTimeout(1));


        NamedCommands.registerCommand("Align To Left Tag", sDrive.getAlignLeftReef(sDrive.getPose()));



        NamedCommands.registerCommand("Move QuickDraw", new RunCommand(
                () -> sQuickDraw.setSpeedFromElevatorPosition(sElevator.getPosition(), 0),
                sQuickDraw).withTimeout(0.5));

        autoChooser = AutoBuilder.buildAutoChooser("None");
        SmartDashboard.putData("Auto Mode", autoChooser);


        configure2JoystickDefaultBindings();
        configure2JoystickBindings();

        // configure1JoystickDefaultBindings();
        // configure1JoystickBindings():

        sDrive.seedFieldCentric();
    }

    private void configure2JoystickDefaultBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        sDrive.setDefaultCommand(
                // Drivetrain will execute this command periodically
                sDrive.applyRequest(() -> drive
                        .withVelocityX(
                                -joystick.getLeftY() * TunerConstants.MaxSpeed * (joystick.getRightTriggerAxis() / 2 + 0.2))
                        .withVelocityY(
                                -joystick.getLeftX() * TunerConstants.MaxSpeed * (joystick.getRightTriggerAxis() / 2 + 0.2))
                        .withRotationalRate(-joystick.getRightX()
                                * TunerConstants.MaxAngularRate)));

        sElevator.setDefaultCommand(
                new RunCommand(
                        () -> sElevator.moveToSetpointWithCounter(),
                        sElevator));

        sSpaceGun.setDefaultCommand(
                new RunCommand(
                        () -> sSpaceGun.shootMotor((joystick2.getLeftY() / 1.5) - 0.01), sSpaceGun));

        sQuickDraw.setDefaultCommand(
                new RunCommand(
                        () -> sQuickDraw.setSpeedFromElevatorPosition(sElevator.getPosition(), joystick2.getRightY()),
                        sQuickDraw));

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
                                joystick2.povDown().getAsBoolean(),
                                joystick2.povDownRight().getAsBoolean(),
                                joystick2.povDownLeft().getAsBoolean(),
                                joystick2.povUp().getAsBoolean(),
                                joystick2.povUpRight().getAsBoolean(),
                                joystick2.povUpLeft().getAsBoolean()),
                        sAlgaer));
                        
        sLED.setDefaultCommand(
                new RunCommand(
                        () -> sLED.runPattern(LEDPattern.solid(sLED.getAllianceColor()).atBrightness(Percent.of(50))
                                .breathe(Seconds.of(5))),
                        sLED).ignoringDisable(true));

        sClimber.setDefaultCommand(
                new RunCommand(
                        () -> sClimber.setMotor(
                                joystick.x().getAsBoolean(),
                                joystick.b().getAsBoolean()),
                        sClimber));
    }

    private void configure1JoystickDefaultBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        sDrive.setDefaultCommand(
                // Drivetrain will execute this command periodically
                sDrive.applyRequest(() -> drive
                        .withVelocityX(
                                -joystick.getLeftY() * TunerConstants.MaxSpeed * (joystick.getLeftTriggerAxis() / 2 + 0.2))
                        .withVelocityY(
                                -joystick.getLeftX() * TunerConstants.MaxSpeed * (joystick.getLeftTriggerAxis() / 2 + 0.2))
                        .withRotationalRate(-joystick.getRightX()
                                * TunerConstants.MaxAngularRate)));

        sElevator.setDefaultCommand(
                new RunCommand(
                        () -> sElevator.moveToSetpointWithCounter(),
                        sElevator));

        sSpaceGun.setDefaultCommand(
                new RunCommand(
                        () -> sSpaceGun.shootMotor((joystick2.getLeftY() / 1.5) - 0.01), sSpaceGun));

        sQuickDraw.setDefaultCommand(
                new RunCommand(
                        () -> sQuickDraw.setSpeedFromElevatorPosition(sElevator.getPosition(), joystick.getRightTriggerAxis() - joystick.getLeftTriggerAxis()),
                        sQuickDraw));

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
                                joystick.povLeft().getAsBoolean(),
                                joystick.povRight().getAsBoolean(),
                                joystick.povDown().getAsBoolean(),
                                joystick.povDownRight().getAsBoolean(),
                                joystick.povDownLeft().getAsBoolean(),
                                joystick.povUp().getAsBoolean(),
                                joystick.povUpRight().getAsBoolean(),
                                joystick.povUpLeft().getAsBoolean()),
                        sAlgaer));
                        
        sLED.setDefaultCommand(
                new RunCommand(
                        () -> sLED.runPattern(LEDPattern.solid(sLED.getAllianceColor()).atBrightness(Percent.of(50))
                                .breathe(Seconds.of(5))),
                        sLED).ignoringDisable(true));

        sClimber.setDefaultCommand(
                new RunCommand(
                        () -> sClimber.setMotor(
                                joystick.x().getAsBoolean(),
                                joystick.b().getAsBoolean()),
                        sClimber));
    }

    private void configure2JoystickBindings() {

        joystick.povRight().whileTrue(sDrive.applyRequest(() -> driveRobotCentric.withVelocityY(-0.05 * TunerConstants.MaxSpeed).withVelocityX(0.0)));
        joystick.povLeft().whileTrue(sDrive.applyRequest(() -> driveRobotCentric.withVelocityY(0.05 * TunerConstants.MaxSpeed).withVelocityX(0.0)));
        joystick.povUp().whileTrue(sDrive.applyRequest(() -> driveRobotCentric.withVelocityX(0.05 * TunerConstants.MaxSpeed).withVelocityY(0.0)));
        joystick.povDown().whileTrue(sDrive.applyRequest(() -> driveRobotCentric.withVelocityX(-0.05 * TunerConstants.MaxSpeed).withVelocityY(0.0)));
        joystick.povUpLeft().whileTrue(sDrive.applyRequest(() -> driveRobotCentric.withVelocityX(0.05 * TunerConstants.MaxSpeed).withVelocityY(0.05 * TunerConstants.MaxSpeed)));
        joystick.povUpRight().whileTrue(sDrive.applyRequest(() -> driveRobotCentric.withVelocityX(0.05 * TunerConstants.MaxSpeed).withVelocityY(-0.05 * TunerConstants.MaxSpeed)));
        joystick.povDownLeft().whileTrue(sDrive.applyRequest(() -> driveRobotCentric.withVelocityX(-0.05 * TunerConstants.MaxSpeed).withVelocityY(0.05 * TunerConstants.MaxSpeed)));
        joystick.povDownRight().whileTrue(sDrive.applyRequest(() -> driveRobotCentric.withVelocityX(-0.05 * TunerConstants.MaxSpeed).withVelocityY(-0.05 * TunerConstants.MaxSpeed)));

        joystick.y().onTrue(sDrive.runOnce(() -> sDrive.seedFieldCentric()));
        joystick.rightBumper().whileTrue(Commands.defer(() -> sDrive.getAlignRightReef(sDrive.getPose()), Set.of(sDrive)));
        joystick.leftBumper().whileTrue(Commands.defer(() -> sDrive.getAlignLeftReef(sDrive.getPose()), Set.of(sDrive)));

        joystick2.povUp().onFalse(new InstantCommand(() -> sAlgaer.setDesiredPosition()));
        joystick2.povUpRight().onFalse(new InstantCommand(() -> sAlgaer.setDesiredPosition()));
        joystick2.povUpLeft().onFalse(new InstantCommand(() -> sAlgaer.setDesiredPosition()));
        joystick2.povDown().onFalse(new InstantCommand(() -> sAlgaer.setDesiredPosition()));
        joystick2.povDownLeft().onFalse(new InstantCommand(() -> sAlgaer.setDesiredPosition()));
        joystick2.povDownRight().onFalse(new InstantCommand(() -> sAlgaer.setDesiredPosition()));

        joystick2.y().onTrue(new InstantCommand(() -> sElevator.incrementCounter()));
        joystick2.a().onTrue(new InstantCommand(() -> sElevator.decrementCounter()));

        joystick2.back().onTrue(putCoralOnSpaceGunCommand());

        sClimber.getSwitch().whileTrue(new RunCommand(() -> sLED.runPattern(LEDPattern.solid(Color.kGreen).atBrightness(Percent.of(25)))));

        joystick2.start().onTrue(pipeScoreCommand());

        sDrive.registerTelemetry(logger::telemeterize);
    }

    private void configure1JoystickBindings() {

        // joystick.povRight().whileTrue(sDrive.applyRequest(() -> driveRobotCentric.withVelocityY(-0.05 * TunerConstants.MaxSpeed).withVelocityX(0.0)));
        // joystick.povLeft().whileTrue(sDrive.applyRequest(() -> driveRobotCentric.withVelocityY(0.05 * TunerConstants.MaxSpeed).withVelocityX(0.0)));
        // joystick.povUp().whileTrue(sDrive.applyRequest(() -> driveRobotCentric.withVelocityX(0.05 * TunerConstants.MaxSpeed).withVelocityY(0.0)));
        // joystick.povDown().whileTrue(sDrive.applyRequest(() -> driveRobotCentric.withVelocityX(-0.05 * TunerConstants.MaxSpeed).withVelocityY(0.0)));
        // joystick.povUpLeft().whileTrue(sDrive.applyRequest(() -> driveRobotCentric.withVelocityX(0.05 * TunerConstants.MaxSpeed).withVelocityY(0.05 * TunerConstants.MaxSpeed)));
        // joystick.povUpRight().whileTrue(sDrive.applyRequest(() -> driveRobotCentric.withVelocityX(0.05 * TunerConstants.MaxSpeed).withVelocityY(-0.05 * TunerConstants.MaxSpeed)));
        // joystick.povDownLeft().whileTrue(sDrive.applyRequest(() -> driveRobotCentric.withVelocityX(-0.05 * TunerConstants.MaxSpeed).withVelocityY(0.05 * TunerConstants.MaxSpeed)));
        // joystick.povDownRight().whileTrue(sDrive.applyRequest(() -> driveRobotCentric.withVelocityX(-0.05 * TunerConstants.MaxSpeed).withVelocityY(-0.05 * TunerConstants.MaxSpeed)));

        joystick.x().onTrue(sDrive.runOnce(() -> sDrive.seedFieldCentric()));
        joystick.rightBumper().whileTrue(Commands.defer(() -> sDrive.getAlignRightReef(sDrive.getPose()), Set.of(sDrive)));
        joystick.leftBumper().whileTrue(Commands.defer(() -> sDrive.getAlignLeftReef(sDrive.getPose()), Set.of(sDrive)));

        joystick.povUp().onFalse(new InstantCommand(() -> sAlgaer.setDesiredPosition()));
        joystick.povUpRight().onFalse(new InstantCommand(() -> sAlgaer.setDesiredPosition()));
        joystick.povUpLeft().onFalse(new InstantCommand(() -> sAlgaer.setDesiredPosition()));
        joystick.povDown().onFalse(new InstantCommand(() -> sAlgaer.setDesiredPosition()));
        joystick.povDownLeft().onFalse(new InstantCommand(() -> sAlgaer.setDesiredPosition()));
        joystick.povDownRight().onFalse(new InstantCommand(() -> sAlgaer.setDesiredPosition()));

        joystick.y().onTrue(new InstantCommand(() -> sElevator.incrementCounter()));
        joystick.a().onTrue(new InstantCommand(() -> sElevator.decrementCounter()));

        joystick.back().onTrue(putCoralOnSpaceGunCommand());


        sClimber.getSwitch().whileTrue(new RunCommand(() -> sLED.runPattern(LEDPattern.solid(Color.kGreen).atBrightness(Percent.of(25)))));

        joystick.start().onTrue(pipeScoreCommand());

        joystick.leftStick().whileTrue(new RunCommand(() -> sSpaceGun.shootMotor(0.06)));
        joystick.rightStick().whileTrue(new RunCommand(() -> sSpaceGun.shootMotor(-0.06)));

        sDrive.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Command troughScoreCommand() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new RunCommand(() -> sQuickDraw
                                        .setSpeedFromElevatorPosition(-1, 0),
                                        sQuickDraw)
                                        .withTimeout(0.5)),
                        new SequentialCommandGroup(
                                new WaitCommand(0.25),
                                new InstantCommand(() -> sSpaceGun.shootMotor(0.4),
                                        sSpaceGun),
                                new WaitCommand(1.0),
                                new InstantCommand(() -> sSpaceGun.shootMotor(0),
                                        sSpaceGun))),
                new RunCommand(() -> sQuickDraw.setSpeedFromElevatorPosition(0, 0), sQuickDraw)
                        .withTimeout(0.75));
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
                                .setSpeedFromElevatorPosition(0, 0),
                                sQuickDraw),
                        new SequentialCommandGroup(
                                new WaitCommand(0.25),
                                new InstantCommand(() -> sElevator.decrementCounter()),
                                new InstantCommand(() -> sElevator.decrementCounter()),
                                new InstantCommand(() -> sElevator.decrementCounter()),
                                new RunCommand(() -> sElevator
                                        .moveToSetpointWithCounter(),
                                        sElevator)))
                        .until(sElevator.getBottomLimitSwitch()));

    }


}