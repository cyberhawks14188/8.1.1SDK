package org.firstinspires.ftc.teamcode.DriveCode;

public class SpeedClass {

    Smoothing Smoothing = new Smoothing();
    DirectionCalc DirectionCalc = new DirectionCalc();

    double lastTime = 0;
    double lastPara = 0;
    double lastPerp = 0;
    double posDelta = 0;
    double lastPosDelta = 0;
    public double currentSpeed = 0;
    public double speedPower = 0;
    public double speedP = .002;
    public double speedD = 0.004;
    double speedError = 0;
    public double lastSpeedError = 0;
    public double speedSetMod = 0;

    public void SpeedCalc(double speedSet, double rampupdist ,double rampdowndist, double distfrom, double currentpara, double currentperp, double time){

        posDelta = Math.sqrt(((currentpara - lastPara) * (currentpara - lastPara)) + (currentperp - lastPerp) * (currentperp - lastPerp));


        currentSpeed = Smoothing.SmoothSpeedCalcInput(posDelta/ (time - lastTime));

        if(distfrom < .5){
          speedSetMod = 0;
        //}else if(DirectionCalc.totalDist - distfrom < rampupdist){
          //  speedSetMod = speedSet * Math.abs((DirectionCalc.totalDist - distfrom) / rampupdist);//not calulating correctly
        } else if(distfrom < rampdowndist){
            speedSetMod = speedSet * (Math.abs(distfrom/rampdowndist));
        }else{
            speedSetMod = speedSet;
        }

        speedError = (speedSetMod - currentSpeed);

        speedPower += (speedError * speedP) + ((speedError - lastSpeedError) * speedD);

        lastSpeedError = speedError;
        lastPosDelta = posDelta;
        lastTime = time;
        lastPara = currentpara;
        lastPerp = currentperp;

    }
}
