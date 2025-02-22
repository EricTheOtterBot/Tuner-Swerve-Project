package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private DigitalInput bottomLimitSwitch;
    private DigitalInput topLimitSwitch;

    private SparkMax leftElevatorMotor;
    private SparkMax rightElevatorMotor;

    private RelativeEncoder leftRelativeEncoder;
    private RelativeEncoder rightRelativeEncoder;

    private PIDController elevatorFeedback;
    //private PIDController elevatorFeedbackMini;

    private double speed;
    private double positioning;
    private double positioningRounded;
    private double maximumPosition = 64.0;
    private double pidMovement;
    //private double pidMovementMini;
    private double stillSetpoint = 0.04;
    private double movingSetpoint = 0.9;
    
    public Elevator() {
        bottomLimitSwitch = new DigitalInput(8);
        topLimitSwitch = new DigitalInput(9);

        leftElevatorMotor = new SparkMax(15, MotorType.kBrushless);
        rightElevatorMotor = new SparkMax(16, MotorType.kBrushless);

        leftRelativeEncoder = leftElevatorMotor.getEncoder();
        rightRelativeEncoder = rightElevatorMotor.getEncoder();

        elevatorFeedback = new PIDController(0.1, 0.0, 0.0);
        //elevatorFeedbackMini = new PIDController(0.01, 0.0, 0.0);

    }

    public void moveToSetpoint(boolean up, boolean down) {

        positioning = leftRelativeEncoder.getPosition();
        
        if(positioning >= maximumPosition * 0.83) {
            positioningRounded = maximumPosition;
        } else if(positioning < maximumPosition * 0.83 && positioning >= maximumPosition * 0.40) {
            positioningRounded = maximumPosition * 0.60;
        } else if(positioning < maximumPosition * 0.45 && positioning >= maximumPosition * 0.20) {
            positioningRounded = maximumPosition * 0.34;
        } else {
            positioningRounded = -2.0;
        }

        pidMovement = elevatorFeedback.calculate(positioning, positioningRounded);
        //pidMovementMini = elevatorFeedbackMini.calculate(positioning, positioningRounded);

        if(bottomLimitSwitch.get()) {
            if(down) {
                speed = stillSetpoint;
            } else if(up) {
                speed = movingSetpoint;
            } else {
                speed = stillSetpoint;
            }
        } else if(topLimitSwitch.get()) {
            if(up) {
                speed = stillSetpoint;
            } else if(down) {
                speed = -movingSetpoint / 2;
            } else {
                speed = stillSetpoint;
            }
        } else {
            if(up) {
                speed = movingSetpoint;//0.05 * positioning;
            } else if(down) {

                speed = -movingSetpoint / 2;
            } else {
                speed = pidMovement;
            }
        }

        if(speed > 1) {
            speed = 1;
        } else if(speed < -1) {
            speed = -1;
        }

        leftElevatorMotor.set(speed);
        rightElevatorMotor.set(-speed);
    }

    public double getPosition() {
        return leftRelativeEncoder.getPosition();
    }

    public BooleanSupplier getBottomLimitSwitch() {
        return () -> bottomLimitSwitch.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Bottom Limit Switch", bottomLimitSwitch.get());
        SmartDashboard.putBoolean("Top Limit Switch", topLimitSwitch.get());

        SmartDashboard.putNumber("Alternate Encoder Position L", leftRelativeEncoder.getPosition());
        SmartDashboard.putNumber("Alternate Encoder Velocity L", leftRelativeEncoder.getVelocity());
        SmartDashboard.putNumber("Alternate Encoder Position R", rightRelativeEncoder.getPosition());
        SmartDashboard.putNumber("Alternate Encoder Velocity R", rightRelativeEncoder.getVelocity());

        SmartDashboard.putNumber("Speed Output", speed);
        SmartDashboard.putNumber("Positioning Rounded", positioningRounded);
        SmartDashboard.putNumber("PID Value", pidMovement);
    }

}
