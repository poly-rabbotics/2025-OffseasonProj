package frc.robot;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;

import edu.wpi.first.math.estimator.SteadyStateKalmanFilter;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DifferentialElevator {
    Joystick controller;

    private static final double STEP_PER_DEGREE = 1.0;
    private static final double STEP_PER_INCH = 1.0;

    private static final double PID_P = 0.1;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;

    private static double ELEVATOR_CURRENT;
    private static double ROTATION_CURRENT;

    double ELEVATOR_LOWERED = 0.0;
    double ELEVATOR_RAISED = 100.0;

    double ROTATION_LOWERED;
    double ROTATION_RAISED = 50.0;

    double encoderLeft, encoderRight;

    
    private TalonFX leftMotor;
    private TalonFX rightMotor;
    TalonFXConfiguration leftConfig;
    TalonFXConfiguration rightConfig;

// Left motor ID 10, Right motor ID 11
    public DifferentialElevator(int leftMotorID, int rightMotorID) {
        controller = new Joystick(1);
        leftMotor = new TalonFX(leftMotorID);
        rightMotor = new TalonFX(rightMotorID);

        encoderLeft = leftMotor.getPosition().getValueAsDouble();
        encoderRight = rightMotor.getPosition().getValueAsDouble();

        leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftConfig.Slot0.kP = PID_P;
        leftConfig.Slot0.kI = PID_I;
        leftConfig.Slot0.kD = PID_D;
        leftMotor.getConfigurator().apply(leftConfig);                         


        rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfig.Slot0.kP = PID_P;
        rightConfig.Slot0.kI = PID_I;
        rightConfig.Slot0.kD = PID_D;
        rightMotor.getConfigurator().apply(rightConfig);  
    }
    public int convertHeight(double desiredHeight){
        int steppedHeight = (int) (desiredHeight / STEP_PER_INCH);
        return steppedHeight;
    }
    //desired rotation in degrees
    public int convertRotation(double desiredRotation){
        int steppedRotation = (int) (desiredRotation / STEP_PER_DEGREE);
        return steppedRotation;
    }
    public double getLeftMotor(){
        return leftMotor.getPosition().getValueAsDouble();
    }
    public double getRightMotor(){
        return rightMotor.getPosition().getValueAsDouble();
    }
    public void run(){
        leftMotor.setControl(new PositionDutyCycle(convertRotation(Pose.pivotPos) - convertHeight(Pose.elevatorPos)));
        rightMotor.setControl(new PositionDutyCycle(convertRotation(Pose.pivotPos) + convertHeight(Pose.defaultElevatorPos)));
    }
}
