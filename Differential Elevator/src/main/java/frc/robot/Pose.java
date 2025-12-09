package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pose {
    public static String pose = "";
    public static double elevatorPos;
    public static double pivotPos;
    
    public static double defaultElevatorPos = 0.0;
    public static double defaultPivotPos = 0.0;

    public Pose(){
        pose = "defaultPos";
        elevatorPos = defaultElevatorPos;
        pivotPos = defaultPivotPos;

    }

public static void run(){
    switch(pose){
        case "default":
            elevatorPos = defaultElevatorPos;
                pivotPos = defaultPivotPos;
                break;
        case "positionOne":
            elevatorPos = 0.0;
                pivotPos = 0.0;
                break;
        case "positionTwo":
            elevatorPos = 1.0;
                pivotPos = 1.0;
                break;
        case "positionThree":
            elevatorPos = 2.0;
                pivotPos = 2.0;
                break;
        default:
        break;

    }
}
public double getElevator(){
    return elevatorPos;
}
public double getPivot(){
    return pivotPos;
}
public static void print(){
SmartDashboard.putNumber("Elevator Pos", elevatorPos);
SmartDashboard.putNumber("Pivot Pos", pivotPos);
}
}
