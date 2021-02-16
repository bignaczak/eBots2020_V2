package org.firstinspires.ftc.teamcode;

import android.util.Log;

import java.util.ArrayList;

import static java.lang.String.format;

public class StarterStackObservation {
    private TargetZone.Zone zone;
    private static ArrayList <StarterStackObservation> observations = new ArrayList<>();
    private boolean debugOn = true;
    private String logTag = "EBOTS";

    public StarterStackObservation (TargetZone.Zone z){
        if(debugOn) Log.d(logTag, "Creating StarterStackObservation...");
        this.zone = z;
        observations.add(this);
        if (observations.size() > 100){
            observations.remove(0);
        }
    }
    public static TargetZone.Zone getObservedTarget(){
        boolean debugOn = true;
        int countA = 0;
        int countB = 0;
        int countC = 0;
        TargetZone.Zone zone;
        for (StarterStackObservation o : observations){
            if (o.zone == TargetZone.Zone.A){
                countA++;
            } else if (o.zone == TargetZone.Zone.B){
                countB++;
            } else {
                countC++;
            }
        }
        if (countA > countB & countA > countC){
                zone = TargetZone.Zone.A;
        } else if (countB > countA & countB > countC) {
            zone = TargetZone.Zone.B;
        } else {
            zone = TargetZone.Zone.C;
        }
        if (debugOn) Log.d("EBOTS", "Observations A/B/C: " + countA +
                " / " + countB + " / " + countC);
        return zone;
    }
}
