package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class QuickDraw extends SubsystemBase {

    private SparkMax quickDrawMotor;
    private PIDController positionController;
    private RelativeEncoder encoder;

    private double encoderPosition;
    private double desiredChange;
    private double desiredValue;
    private int desiredPosition;
    private int position;

    public QuickDraw() {
        quickDrawMotor = new SparkMax(11, MotorType.kBrushless);
        positionController = new PIDController(0.1, 0.0, 0.0);
    }

    public int elevatorToQuickDraw(double elevatorPosition) {
        if(elevatorPosition >= 50) {
            position = 3;
        } else if(elevatorPosition < 50 && elevatorPosition >= 30) {
            position = 2;
        } else if(elevatorPosition < 30 && elevatorPosition >= 10) {
            position = 1;
        } else {
            position = 0;
        }
        return position;
    }

    public void setPosition(double elevatorPosition) {
        desiredPosition = elevatorToQuickDraw(elevatorPosition);
        encoderPosition = encoder.getPosition();
        if(desiredPosition == 3) {
            desiredValue = 5;
        } else if(desiredPosition == 2) {
            desiredValue = 3.5;
        } else if(desiredPosition == 1) {
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
