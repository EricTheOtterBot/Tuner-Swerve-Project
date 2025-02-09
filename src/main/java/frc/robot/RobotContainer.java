package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Extras.Telemetry;
import frc.robot.Extras.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.QuickDraw;
import frc.robot.subsystems.SpaceGun;

public class RobotContainer {
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.MaxSpeed * 0.02).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.02) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentric driveDetailed = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(TunerConstants.MaxSpeed * 0.02).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.02)
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

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Test Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
        drivetrain.seedFieldCentric();

        NamedCommands.registerCommand("Align To Tag", null);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * TunerConstants.MaxSpeed / 3) // Drive forward with negative Y (forward)
                        .withVelocityY(-joystick.getLeftX() * TunerConstants.MaxSpeed / 3) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * TunerConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        sElevator.setDefaultCommand(
                new RunCommand(
                        () -> sElevator.moveToSetpoint(
                                joystick2.y().getAsBoolean(),
                                joystick2.a().getAsBoolean()),
                        sElevator));

        sSpaceGun.setDefaultCommand(
                new RunCommand(
                        () -> sSpaceGun.shootMotor(joystick2.getLeftY()), sSpaceGun));

        sQuickDraw.setDefaultCommand(
                new RunCommand(
                        () -> sQuickDraw.setVelocity(joystick2.getRightY()), sQuickDraw));

        sIndexer.setDefaultCommand(
            new RunCommand(() -> 
            sIndexer.setFingerAndCannon(
                joystick2.x().getAsBoolean(), 
                joystick2.b().getAsBoolean(), 
                joystick2.getLeftTriggerAxis(), 
                joystick2.getRightTriggerAxis()),
                 sIndexer)
        );

        joystick.leftBumper().whileTrue(drivetrain.applyRequest(() -> driveDetailed
                .withVelocityX(drivetrain.getPoseForAlign().getX() * TunerConstants.MaxSpeed)
                .withVelocityY(drivetrain.getPoseForAlign().getY() * TunerConstants.MaxSpeed)
                .withRotationalRate(
                        drivetrain.getPoseForAlign().getRotation().getRadians() * TunerConstants.MaxAngularRate)));

        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> driveRobotCentric
                .withVelocityY(-0.05 * TunerConstants.MaxSpeed)));

        joystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
