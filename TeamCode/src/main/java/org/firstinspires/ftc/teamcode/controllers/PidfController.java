package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PidfController
{
    private ElapsedTime runtime = new ElapsedTime();
    private double kP, kI, kD, kF, iZone;
    private double prevErr, totalErr, error;
    private Double inputMin = null, inputMax = null, inputModulus = null, inputBound = null;


    public PidfController(double kP, double kI, double kD, double kF, double iZone)
    {
        setPidCoefficients(kP, kI, kD, kF, iZone);
        reset();
    }

    public PidfController(double kP, double kI, double kD)
    {
        this(kP, kI, kD, 0.0, 0.0);
    }

    public void setPidCoefficients(double kP, double kI, double kD, double kF, double iZone)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.iZone = iZone;
    }

    public void setPidCoefficients(double kP, double kI, double kD)
    {
        setPidCoefficients(kP, kI, kD, 0.0, 0.0);
    }


    public void reset()
    {
        prevErr = totalErr = 0.0;
        runtime.reset();
    }

    public void enableWrapInput(double inputMin, double inputMax)
    {
        this.inputMin = inputMin;
        this.inputMax = inputMax;
        this.inputModulus = inputMax - inputMin;
        this.inputBound = inputModulus / 2.0;
    }

    public void disableWrapInput()
    {
        inputMin = inputMax = inputModulus = inputBound = null;
    }

    public boolean isOnTarget(double tolerance)
    {
        return Math.abs(prevErr) <= tolerance;
    }

    public double getError() {
        return error;
    }

    public double calculate(double reference, double input)
    {
        double deltaTime = runtime.seconds();
        error = reference - input;
        runtime.reset();
        // For wrap input, calculate shortest route to setPoint.
        if (inputBound != null)
        {
            error = inputMod(error, -inputBound, inputBound);
        }

        if (Math.abs(error) > iZone)
        {
            totalErr = 0.0;
        }
        else if (kI != 0.0)
        {
            totalErr += error * deltaTime;
        }
        double pTerm = kP * error;
        double iTerm = kI * totalErr;
        double dTerm = kD * (error - prevErr) / deltaTime;
        double fTerm = kF * reference;
        double output = Range.clip(pTerm + iTerm + dTerm + fTerm, -1.0, 1.0);

        return output;
    }

    private double inputMod(double input, double lowerBound, double upperBound)
    {
        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - lowerBound) / inputModulus);
        input -= numMax * inputModulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - upperBound) / inputModulus);
        input -= numMin * inputModulus;

        return input;
    }
}