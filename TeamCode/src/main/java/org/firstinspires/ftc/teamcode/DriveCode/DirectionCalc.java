package org.firstinspires.ftc.teamcode.DriveCode;

public class DirectionCalc {

    public double Y1 = 0, Y2 = 0;
    public double X1 = 0, X2 = 0;
    public double slope = 0;
    public double distanceFrom = 1.1;
    public double setX = 0, setY = 0;
    public double directionVector = 0;
    public double paradistfrom = 0;
    public boolean hasDistFrom = false;
    double lastEndSetPara = 0;
    double lastEndSetPerp = 0;
    boolean oneloop = false;
    public double totalDist = 0;



    public void DirectionMethod(double endSetPara, double endSetPerp, double startPara, double startPerp,double currentPara, double currentPerp){

        if(oneloop){
            if(lastEndSetPerp != endSetPerp){
                hasDistFrom = false;
            }
            if(lastEndSetPara != endSetPara){
                hasDistFrom = false;
            }
        }


        Y1 = endSetPara - startPara;
        X1 = endSetPerp - startPerp;
        slope = Math.atan2(X1, Y1); //IN RADIANS

        paradistfrom = endSetPara;

        distanceFrom = Math.sqrt( (endSetPara - currentPara) * (endSetPara - currentPara) + (endSetPerp - currentPerp) * (endSetPerp - currentPerp));

        totalDist = Math.sqrt(((endSetPara - startPara) * (endSetPara - startPara)) + ((endSetPerp - startPerp) * (endSetPerp - startPerp)));

        if(distanceFrom > 1){
            Y2 = (distanceFrom - 1) * Math.cos(slope);
            X2 = (distanceFrom - 1) * Math.sin(slope);

            setY = endSetPara - Y2;
            setX = endSetPerp - X2;
        }else{
            hasDistFrom = true;
        }

        if(hasDistFrom){
            setY = endSetPara;
            setX = endSetPerp;
        }



        directionVector = Math.toDegrees(Math.atan2(setX-currentPerp, setY - currentPara));// + 90;

        if(directionVector < 0){
            directionVector += 360;
        }else if(directionVector > 360){
            directionVector -= 360;
        }
        lastEndSetPara = endSetPara;
        lastEndSetPerp = endSetPerp;

        oneloop = true;
    }
}
