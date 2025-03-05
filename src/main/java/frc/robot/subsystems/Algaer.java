package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algaer extends SubsystemBase {

    private SparkMax rotationMotor;
    private SparkMax shooterMotor; 
    private RelativeEncoder encoder;
    private double desiredPosition;
    private double speed;

    private PIDController rotationController;

    public Algaer() {
        rotationMotor = new SparkMax(14, MotorType.kBrushless);
        shooterMotor = new SparkMax(13, MotorType.kBrushless);

        encoder = rotationMotor.getEncoder();

        rotationController = new PIDController(0.05, 0.0, 0.0);
    }

    public void setMotors(
        boolean shooterOut, 
        boolean shooterIn, 
        boolean rotationOut1, 
        boolean rotationOut2, 
        boolean rotationOut3, 
        boolean rotationIn1, 
        boolean rotationIn2, 
        boolean rotationIn3) {
        if(shooterOut) {
            shooterMotor.set(0.3);
        } else if(shooterIn) {
            shooterMotor.set(-1);
        } else {
            if(encoder.getPosition() < -1) {
                shooterMotor.set(0.02);
            } else {
                shooterMotor.set(0);
            }
        }

        if(rotationOut1 || rotationOut2 || rotationOut3) {
            if(encoder.getPosition() < -46) {
                rotationMotor.set(0.0);
            } else {
                rotationMotor.set(-0.5);
            }
        } else if(rotationIn1 || rotationIn2 || rotationIn3) {
            if(encoder.getPosition() > 0) {
                rotationMotor.set(0.0);
            } else {
                rotationMotor.set(0.5);
            }
        } else {
            speed = rotationController.calculate(encoder.getPosition(), desiredPosition);
            rotationMotor.set(speed);
        }
    }

    public void setDesiredPosition() {
        desiredPosition = encoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Algaer", encoder.getPosition());
    }
}