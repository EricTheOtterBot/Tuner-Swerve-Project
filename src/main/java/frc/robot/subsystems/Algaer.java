package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algaer extends SubsystemBase {

    private SparkMax rotationMotor;
    private SparkMax shooterMotor; 
    private RelativeEncoder encoder;

    public Algaer() {
        rotationMotor = new SparkMax(14, MotorType.kBrushless);
        shooterMotor = new SparkMax(13, MotorType.kBrushless);

        encoder = rotationMotor.getEncoder();
    }

    public void setMotors(boolean shooterOut, boolean shooterIn, boolean rotationOut, boolean rotationIn) {
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

        if(rotationOut) {
            if(encoder.getPosition() < -46) {
                rotationMotor.set(0.0);
            } else {
                rotationMotor.set(-0.3);
            }
        } else if(rotationIn) {
            if(encoder.getPosition() > 0) {
                rotationMotor.set(0.0);
            } else {
                rotationMotor.set(0.3);
            }
        } else {
            rotationMotor.set(0);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Algaer", encoder.getPosition());
    }
}