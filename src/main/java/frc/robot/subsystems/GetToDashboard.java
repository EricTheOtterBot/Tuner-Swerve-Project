package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GetToDashboard extends SubsystemBase {

    private boolean eric1;
    private boolean eric2;
    private boolean eric3;
    private boolean eric4;
    private boolean eric5;
    private boolean eric6;
    private boolean eric7;
    private boolean eric8;
    private boolean eric9;
    private boolean eric10;
    private boolean eric11;
    private boolean eric12;
    private boolean eric13;
    private boolean eric14;
    private boolean eric15;
    private boolean eric16;
    private boolean eric17;
    private boolean eric18;
    private boolean eric19;
    private boolean eric20;

    public GetToDashboard() {
        eric1 = false;
        eric2 = false;
        eric3 = false;
        eric4 = false;
        eric5 = false;
        eric6 = false;
        eric7 = false;
        eric8 = false;
        eric9 = false;
        eric10 = false;
        eric11 = false;
        eric12 = false;
        eric13 = false;
        eric14 = false;
        eric15 = false;
        eric16 = false;
        eric17 = false;
        eric18 = false;
        eric19 = false;
        eric20 = false;
    }

    public void getValues( 
    boolean edawg1,
    boolean edawg2, 
    boolean edawg3, 
    boolean edawg4, 
    boolean edawg5, 
    boolean edawg6, 
    boolean edawg7, 
    boolean edawg8, 
    boolean edawg9, 
    boolean edawg10, 
    boolean edawg11, 
    boolean edawg12, 
    boolean edawg13, 
    boolean edawg14, 
    boolean edawg15, 
    boolean edawg16, 
    boolean edawg17, 
    boolean edawg18, 
    boolean edawg19, 
    boolean edawg20) {

    eric1 = edawg1;
    eric2 = edawg2;
    eric3 = edawg3;
    eric4 = edawg4;
    eric5 = edawg5;
    eric6 = edawg6;
    eric7 = edawg7;
    eric8 = edawg8; 
    eric9 = edawg9;
    eric10 = edawg10;
    eric11 = edawg11;
    eric12 = edawg12;
    eric13 = edawg13;
    eric14 = edawg14;
    eric15 = edawg15;
    eric16 = edawg16;
    eric17 = edawg17;
    eric18 = edawg18;
    eric19 = edawg19;
    eric20 = edawg20;

    SmartDashboard.putBoolean("Button 1", eric1);
    SmartDashboard.putBoolean("Button 2", eric2);
    SmartDashboard.putBoolean("Button 3", eric3);
    SmartDashboard.putBoolean("Button 4", eric4);
    SmartDashboard.putBoolean("Button 5", eric5);
    SmartDashboard.putBoolean("Button 6", eric6);
    SmartDashboard.putBoolean("Button 7", eric7);
    SmartDashboard.putBoolean("Button 8", eric8);
    SmartDashboard.putBoolean("Button 9", eric9);
    SmartDashboard.putBoolean("Button 10", eric10);
    SmartDashboard.putBoolean("Button 11", eric11);
    SmartDashboard.putBoolean("Button 12", eric12);
    SmartDashboard.putBoolean("Button 13", eric13);
    SmartDashboard.putBoolean("Button 14", eric14);
    SmartDashboard.putBoolean("Button 15", eric15);
    SmartDashboard.putBoolean("Button 16", eric16);
    SmartDashboard.putBoolean("Button 17", eric17);
    SmartDashboard.putBoolean("Button 18", eric18);
    SmartDashboard.putBoolean("Button 19", eric19);
    SmartDashboard.putBoolean("Button 20", eric20);
        
    }    
}
