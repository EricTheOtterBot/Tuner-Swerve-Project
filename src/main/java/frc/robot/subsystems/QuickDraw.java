package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class QuickDraw extends SubsystemBase {

    private SparkMax quickDrawMotor;
    private PIDController positionController;
    private RelativeEncoder encoder;

    private double desiredValue;

    public QuickDraw() {
        quickDrawMotor = new SparkMax(11, MotorType.kBrushless);
        positionController = new PIDController(0.06, 0.0, 0.0);
        encoder = quickDrawMotor.getEncoder();
    }

    public void setSpeedFromElevatorPosition(double elevatorPosition) {
        if(elevatorPosition >= 50) {
            desiredValue = -12;
        } else if(elevatorPosition < 50 && elevatorPosition >= 10) {
            desiredValue = -10;
        } else if(elevatorPosition == -1) {
            desiredValue = -23;
        } else {
            desiredValue = -1;
        }
        quickDrawMotor.set(positionController.calculate(encoder.getPosition(), desiredValue));
        SmartDashboard.putNumber("What Do I Want", positionController.calculate(encoder.getPosition(), desiredValue));
    }

    public void setVelocity(double speed) {
        if(encoder.getPosition() < -22) {
            if(speed < 0) {
                speed = 0;
            }
        }
        if(encoder.getPosition() > 2) {
            if(speed > 0) {
                speed = 0;
            }
        }
        quickDrawMotor.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Encoder for QuickDraw", encoder.getPosition());
    }
    
}
