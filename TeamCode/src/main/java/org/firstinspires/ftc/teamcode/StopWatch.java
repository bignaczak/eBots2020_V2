package org.firstinspires.ftc.teamcode;

import static java.lang.String.format;

public class StopWatch {
    private long startTime;

    //This constructor was refactored 11/11 to start the timer during instantiation
    public StopWatch(){
        startTime = System.currentTimeMillis();
    }

    public void startTimer(){
        startTime = System.currentTimeMillis();  //current time in milliseconds
    }

    public void reset(){
        this.startTimer();
    }

    public long getElapsedTimeMillis(){
        return startTime == 0 ? 0L : System.currentTimeMillis() - startTime;
    }

    public double getElapsedTimeSeconds(){
        return startTime == 0 ? 0.0 : ((System.currentTimeMillis() - startTime) / 1000.0);
    }

    @Override
    public String toString(){
        return String.format("%.2f", getElapsedTimeSeconds()) + " seconds";
    }

    public String toString(Integer loopCount){
        Double frequency = loopCount / getElapsedTimeSeconds();
        return loopCount + " loops in " + getElapsedTimeMillis() + " ms or " + String.format("%.2f", frequency) + " Hz";
    }

}

