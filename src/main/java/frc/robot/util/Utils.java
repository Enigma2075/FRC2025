package frc.robot.util;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.RobotConstants;

public class Utils {
    public static int getValue(int prac, int comp){
       if(RobotConstants.kPracticeBot) {
        return prac;
       }
       return comp;
    }

    public static double getValue(double prac, double comp){
        if(RobotConstants.kPracticeBot) {
         return prac;
        }
        return comp;
     }

     public static String getValue(String prac, String comp){
        if(RobotConstants.kPracticeBot) {
         return prac;
        }
        return comp;
     }

     public static Angle getValue(Angle prac, Angle comp){
        if(RobotConstants.kPracticeBot) {
         return prac;
        }
        return comp;
     }
     
}
