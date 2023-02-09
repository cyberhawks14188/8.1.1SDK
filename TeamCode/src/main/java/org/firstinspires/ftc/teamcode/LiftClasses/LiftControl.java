package org.firstinspires.ftc.teamcode.LiftClasses;

public class LiftControl {
    public double liftpower = 0;
    double currentTime = 0, lastTime = 0;
    double liftPos = 0, lastliftpos = 0, liftSet, liftCalPower = 0;
    double deltaLiftPos = 0;
    public double liftDirection = 1;
    double liftSpeedSet = 0;
    public double liftSpeed, lastLiftSpeed, liftSpeedDifference = 0;
    public double liftP = .0003;

    public void LiftMethod(double liftset, double liftspeedset, double liftcurrentpos, double time){

        currentTime = time;
        liftPos = liftcurrentpos;

        if(liftset > 1150){
            liftSet = 1150;
        }else if(liftset < 0){
            liftSet = 0;
        }else{
            liftSet = liftset;
        }



        deltaLiftPos = liftPos - lastliftpos;

        if(liftSet > liftPos){
            liftDirection = 1;
        }else if(liftSet < liftPos){
            liftDirection = -1;
        }

        liftSpeedSet = Math.copySign(liftspeedset, liftDirection);

        if(Math.abs(liftcurrentpos - liftSet) < 200){
            liftSpeedSet = liftSpeedSet * ((Math.abs(liftcurrentpos - liftSet))/200);
        }

        liftSpeed = deltaLiftPos/(currentTime-lastTime);

        liftSpeedDifference = liftSpeedSet - liftSpeed;

        liftCalPower += (liftSpeedDifference * liftP);
        if(liftCalPower > 1){
            liftCalPower = 1;
        }else if(liftCalPower < -1){
            liftCalPower = -1;
        }

        liftpower = liftCalPower;

        if(liftpower > 1){
            liftpower = 1;
        }else if(liftpower < -1){
            liftpower = -1;
        }

        lastLiftSpeed = liftSpeed;
        lastliftpos = liftPos;
        lastTime = currentTime;

    }
}
