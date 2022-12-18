package frc.robot.subsystems;

import org.opencv.core.Mat;

public class PIDController {

    double P, I, D;

    double setpoint, prevSetpoint;

    double integral = 0;

    double error, prevError;

    public  PIDController (double P, double I, double D){
        this.P = P;
        this.I = I;
        this.D = D;
    }

    public void setSetpoint(double setpoint){

        prevSetpoint = setpoint;
        this.setpoint = setpoint;


    }

    public double PID(double input){
        error = setpoint - input; // Error = Target - Actual

        if (Math.abs(prevSetpoint - setpoint) > 0.2){
            integral = 0;
        }
        integral += (error*.02);
        double derivative = (error - prevError) / .02;

        prevError = error;


        return P*error + I*this.integral + D*derivative;



    }


}
