package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;

public class QuickDraw {

    private SparkMax quickDrawMotor;
    private PIDController positionController;
    private RelativeEncoder encoder;

    private double encoderPosition;
    private double desiredChange;
    private double desiredValue;
    private int desiredPosition;
    private int position;

    public QuickDraw() {
        quickDrawMotor = new SparkMax(12, MotorType.kBrushless);
    }

    public int elevatorToQuickDraw(double elevatorPosition) {
        if(elevatorPosition >= 50) {
            desiredPosition = 3;
        } else if(elevatorPosition < 50 && elevatorPosition >= 30) {
            desiredPosition = 2;
        } else if(elevatorPosition < 30 && elevatorPosition >= 10) {
            desiredPosition = 1;
        } else {
            desiredPosition = 0;
        }
        return desiredPosition;
    }

    public void setPosition(double elevatorPosition) {
        position = elevatorToQuickDraw(elevatorPosition);
        encoderPosition = encoder.getPosition();
        if(position == 3) {
            desiredValue = 5;
        } else if(position == 2) {
            desiredValue = 3.5;
        } else if(position == 1) {
            desiredValue = 2;
        } else {
            desiredValue = -0.5;
        }
        desiredChange = positionController.calculate(encoderPosition, desiredValue);
        quickDrawMotor.set(desiredChange);
    }
    public void setVelocity(double speed) {
        quickDrawMotor.set(speed);
    }
    
}
