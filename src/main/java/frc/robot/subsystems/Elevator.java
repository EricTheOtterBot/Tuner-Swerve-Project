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

    private RelativeEncoder leftRelativeEncoder;

    private PIDController elevatorFeedback;

    private double speed;
    private double positioning;
    private double positioningRounded;
    private double maximumPosition = 64.0;
    private double pidMovement;
    private double stillSetpoint = 0.04;
    private double movingSetpoint = 0.5;

    private int counter = 0;
    
    public Elevator() {
        bottomLimitSwitch = new DigitalInput(8);
        topLimitSwitch = new DigitalInput(9);

        leftElevatorMotor = new SparkMax(15, MotorType.kBrushless);

        leftRelativeEncoder = leftElevatorMotor.getEncoder();

        elevatorFeedback = new PIDController(0.1, 0.0, 0.0);

    }

    public void incrementCounter() {
        if(counter >= 3) {
            counter = 3;
        } else {
            counter++;
        }
    }

    public void decrementCounter() {
        if(counter <= 0) {
            counter = 0;
        } else {
            counter--;
        }
    }
    
    public void moveToSetpointWithCounter() {
        positioning = leftRelativeEncoder.getPosition();

        if(counter == 0) {

        }
        switch(counter) {
            case 0: positioningRounded = -1.0;

            case 1: positioningRounded = maximumPosition * 0.25;

            case 2: positioningRounded = maximumPosition * 0.54;

            case 3: positioningRounded = maximumPosition;
        }

        pidMovement = elevatorFeedback.calculate(positioning, positioningRounded);

        //leftElevatorMotor.set(pidMovement);

        SmartDashboard.putNumber("pidMovement", pidMovement);

        
    }

    public void moveToSetpoint(boolean up, boolean down) {

        positioning = leftRelativeEncoder.getPosition();
        
        if(positioning >= maximumPosition * 0.83) {
            positioningRounded = maximumPosition;
        } else if(positioning < maximumPosition * 0.83 && positioning >= maximumPosition * 0.40) {
            positioningRounded = maximumPosition * 0.54;
        } else if(positioning < maximumPosition * 0.45 && positioning >= maximumPosition * 0.12) {
            positioningRounded = maximumPosition * 0.25;
        } else {
            positioningRounded = -2.0;
        }

        pidMovement = elevatorFeedback.calculate(positioning, positioningRounded);

        

        if(bottomLimitSwitch.get()) {
            if(down) {
                speed = 0.0;
            } else if(up) {
                speed = movingSetpoint;
            } else {
                speed = 0.0;
            }
        } else if(topLimitSwitch.get()) {
            if(up) {
                speed = stillSetpoint;
            } else if(down) {
                speed = -movingSetpoint / 3;
            } else {
                speed = stillSetpoint;
            }
        } else {
            if(up) {
                speed = movingSetpoint;
            } else if(down) {

                speed = -0.02 * positioning;
            } else {
                speed = pidMovement;
            }
        }

        if(speed > 0.5) {
            speed = 0.5;
        } else if(speed < -0.5) {
            speed = -0.5;
        }

        leftElevatorMotor.set(speed);
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

        SmartDashboard.putNumber("Speed Output", speed);
        SmartDashboard.putNumber("Positioning Rounded", positioningRounded);
        SmartDashboard.putNumber("PID Value", pidMovement);
    }

}
