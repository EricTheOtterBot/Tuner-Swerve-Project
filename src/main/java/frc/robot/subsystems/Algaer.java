package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algaer extends SubsystemBase {

    private SparkMax rotationMotor;
    private SparkMax shooterMotor; 

    public Algaer() {
        rotationMotor = new SparkMax(14, MotorType.kBrushless);
        shooterMotor = new SparkMax(13, MotorType.kBrushless);
    }

    public void setMotors(boolean shooterOut, boolean shooterIn, boolean rotationOut, boolean rotationIn) {
        if(shooterOut) {
            shooterMotor.set(1);
        } else if(shooterIn) {
            shooterMotor.set(-1);
        } else {
            shooterMotor.set(0);
        }

        if(rotationOut) {
            rotationMotor.set(-0.3);
        } else if(rotationIn) {
            rotationMotor.set(0.3);
        } else {
            rotationMotor.set(0);
        }
        //rotationMotor.set(rotationSpeed);
    }
}