package org.firstinspires.ftc.teamcode.DriveCode;

public class Smoothing {

    double ExampleInputTotal;
    int ExampleInputArrayNum = 0, ExampleInputfirstLoop = 0;
    double ExampleInputArray[] = new double[10];

    public double SmoothExampleInput(double input){
        if(ExampleInputfirstLoop == 0) {
            ExampleInputfirstLoop = 1;
            for (int arrayInitialSet = 0; arrayInitialSet < 10; arrayInitialSet++) {
                ExampleInputArray[arrayInitialSet] = 0;
            }
        }
        ExampleInputTotal = ExampleInputTotal - ExampleInputArray[ExampleInputArrayNum];

        ExampleInputArray[ExampleInputArrayNum] = input;

        ExampleInputTotal = ExampleInputTotal + ExampleInputArray[ExampleInputArrayNum];


        ExampleInputArrayNum = ExampleInputArrayNum + 1;
        if(ExampleInputArrayNum >= 10){
            ExampleInputArrayNum = 0;
        }

        if(Math.abs(input) < .05){
            for (int arrayInitialSet = 0; arrayInitialSet < 10; arrayInitialSet++) {
                ExampleInputArray[arrayInitialSet] = 0;
            }
            ExampleInputTotal = 0;
        }

        return ExampleInputTotal/10;

    }

    double DirectionInputTotal;
    int DirectionInputArrayNum = 0, DirectionInputfirstLoop = 0;
    double DirectionInputArray[] = new double[10];

    public double SmoothDirectionInput(double input){
        if(DirectionInputfirstLoop == 0) {
            DirectionInputfirstLoop = 1;
            for (int arrayInitialSet = 0; arrayInitialSet < 10; arrayInitialSet++) {
                DirectionInputArray[arrayInitialSet] = 0;
            }
        }
        DirectionInputTotal = DirectionInputTotal - DirectionInputArray[DirectionInputArrayNum];

        DirectionInputArray[DirectionInputArrayNum] = input;

        DirectionInputTotal = DirectionInputTotal + DirectionInputArray[DirectionInputArrayNum];


        DirectionInputArrayNum = DirectionInputArrayNum + 1;
        if(DirectionInputArrayNum >= 10){
            DirectionInputArrayNum = 0;
        }

        if(Math.abs(input) < .05){
            for (int arrayInitialSet = 0; arrayInitialSet < 10; arrayInitialSet++) {
                DirectionInputArray[arrayInitialSet] = 0;
            }
            DirectionInputTotal = 0;
        }

        return DirectionInputTotal/10;

    }

    double SpeedCalcInputTotal;
    int SpeedCalcInputArrayNum = 0, SpeedCalcInputfirstLoop = 0;
    double SpeedCalcInputArray[] = new double[4];

    public double SmoothSpeedCalcInput(double input){
        if(SpeedCalcInputfirstLoop == 0) {
            SpeedCalcInputfirstLoop = 1;
            for (int arrayInitialSet = 0; arrayInitialSet < 4; arrayInitialSet++) {
                SpeedCalcInputArray[arrayInitialSet] = 0;
            }
        }
        SpeedCalcInputTotal = SpeedCalcInputTotal - SpeedCalcInputArray[SpeedCalcInputArrayNum];

        SpeedCalcInputArray[SpeedCalcInputArrayNum] = input;

        SpeedCalcInputTotal = SpeedCalcInputTotal + SpeedCalcInputArray[SpeedCalcInputArrayNum];


        SpeedCalcInputArrayNum = SpeedCalcInputArrayNum + 1;
        if(SpeedCalcInputArrayNum >= 4){
            SpeedCalcInputArrayNum = 0;
        }

        if(Math.abs(input) < .05){
            for (int arrayInitialSet = 0; arrayInitialSet < 4; arrayInitialSet++) {
                SpeedCalcInputArray[arrayInitialSet] = 0;
            }
            SpeedCalcInputTotal = 0;
        }

        return SpeedCalcInputTotal/4;

    }

    double PerpendicularInputTotal;
    int PerpendicularInputArrayNum = 0, PerpendicularInputfirstLoop = 0;
    double PerpendicularInputArray[] = new double[10];

    public double SmoothPerpendicularInput(double input){
        if(PerpendicularInputfirstLoop == 0) {
            PerpendicularInputfirstLoop = 1;
            for (int arrayInitialSet = 0; arrayInitialSet < 10; arrayInitialSet++) {
                PerpendicularInputArray[arrayInitialSet] = 0;
            }
        }
        PerpendicularInputTotal = PerpendicularInputTotal - PerpendicularInputArray[PerpendicularInputArrayNum];

        PerpendicularInputArray[PerpendicularInputArrayNum] = input;

        PerpendicularInputTotal = PerpendicularInputTotal + PerpendicularInputArray[PerpendicularInputArrayNum];


        PerpendicularInputArrayNum = PerpendicularInputArrayNum + 1;
        if(PerpendicularInputArrayNum >= 10){
            PerpendicularInputArrayNum = 0;
        }

        if(Math.abs(input) < .05){
            for (int arrayInitialSet = 0; arrayInitialSet < 10; arrayInitialSet++) {
                PerpendicularInputArray[arrayInitialSet] = 0;
            }
            PerpendicularInputTotal = 0;
        }

        return PerpendicularInputTotal/10;

    }
    
    double ParallelInputTotal;
    int ParallelInputArrayNum = 0, ParallelInputfirstLoop = 0;
    double ParallelInputArray[] = new double[10];

    public double SmoothParallelInput(double input){
        if(ParallelInputfirstLoop == 0) {
            ParallelInputfirstLoop = 1;
            for (int arrayInitialSet = 0; arrayInitialSet < 10; arrayInitialSet++) {
                ParallelInputArray[arrayInitialSet] = 0;
            }
        }
        ParallelInputTotal = ParallelInputTotal - ParallelInputArray[ParallelInputArrayNum];

        ParallelInputArray[ParallelInputArrayNum] = input;

        ParallelInputTotal = ParallelInputTotal + ParallelInputArray[ParallelInputArrayNum];


        ParallelInputArrayNum = ParallelInputArrayNum + 1;
        if(ParallelInputArrayNum >= 10){
            ParallelInputArrayNum = 0;
        }

        if(Math.abs(input) < .05){
            for (int arrayInitialSet = 0; arrayInitialSet < 10; arrayInitialSet++) {
                ParallelInputArray[arrayInitialSet] = 0;
            }
            ParallelInputTotal = 0;
        }

        return ParallelInputTotal/10;

    }
    
    double HeadingInputTotal;
    int HeadingInputArrayNum = 0, HeadingInputfirstLoop = 0;
    double HeadingInputArray[] = new double[20];

    public double SmoothHeadingInput(double input){
        if(HeadingInputfirstLoop == 0) {
            HeadingInputfirstLoop = 1;
            for (int arrayInitialSet = 0; arrayInitialSet < 20; arrayInitialSet++) {
                HeadingInputArray[arrayInitialSet] = 0;
            }
        }
        HeadingInputTotal = HeadingInputTotal - HeadingInputArray[HeadingInputArrayNum];

        HeadingInputArray[HeadingInputArrayNum] = input;

        HeadingInputTotal = HeadingInputTotal + HeadingInputArray[HeadingInputArrayNum];


        HeadingInputArrayNum = HeadingInputArrayNum + 1;
        if(HeadingInputArrayNum >= 20){
            HeadingInputArrayNum = 0;
        }

        if(Math.abs(input) < .05){
            for (int arrayInitialSet = 0; arrayInitialSet < 20; arrayInitialSet++) {
                HeadingInputArray[arrayInitialSet] = 0;
            }
            HeadingInputTotal = 0;
        }

        return HeadingInputTotal/20;

    }

}
