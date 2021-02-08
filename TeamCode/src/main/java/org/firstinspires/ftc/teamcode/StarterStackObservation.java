package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class StarterStackObservation {
    private TargetZone.Zone zone;
    private static ArrayList <StarterStackObservation> observations = new ArrayList<>();
    public StarterStackObservation (TargetZone.Zone z){
        this.zone = z;
        observations.add(this);
        if (observations.size() > 100){
            observations.remove(0);
        }
    }
    public static TargetZone.Zone getObservedTarget(){
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
        return zone;
    }
}
