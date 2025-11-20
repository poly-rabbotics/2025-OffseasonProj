package frc.robot;

import static edu.wpi.first.units.Units.Newton;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI {
    DifferentialElevator elevator = Robot.elevator;
    Joystick controller;

    
    boolean[] contPressed = new boolean[13];
    boolean[] contReleased = new boolean[13];
    boolean[] contRaw = new boolean[13];


    public OI()
    {
        controller = new Joystick(1);
    }

    public void run()
    {
        for(int i=1;i<=12;i++)
        {
        contPressed[i] = controller.getRawButtonPressed(i);
        contReleased[i] = controller.getRawButtonReleased(i);
        contRaw[i] = controller.getRawButton(i);
        }
        if(contRaw[1]){ //raise
            elevator.setHeight(elevator.ELEVATOR_RAISED);
            
        }
        else{ //default
            elevator.setHeight(0);
        }
        }
    }
