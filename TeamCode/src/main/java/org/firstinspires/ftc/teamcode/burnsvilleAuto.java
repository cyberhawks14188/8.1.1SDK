package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.DriveCode.DirectionCalc;
import org.firstinspires.ftc.teamcode.DriveCode.HeadingControl;
import org.firstinspires.ftc.teamcode.DriveCode.OdometryCode;
import org.firstinspires.ftc.teamcode.DriveCode.Smoothing;
import org.firstinspires.ftc.teamcode.DriveCode.SpeedClass;
import org.firstinspires.ftc.teamcode.Jake_2_Hardware;
import org.firstinspires.ftc.teamcode.LiftClasses.LiftControl;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous

public class burnsvilleAuto extends LinearOpMode {

    Jake_2_Hardware robot = new Jake_2_Hardware();
    OdometryCode ODO = new OdometryCode();
    LiftControl Lift = new LiftControl();
    HeadingControl HDing = new HeadingControl();
    Smoothing Smoothing = new Smoothing();
    DirectionCalc DirectionCalc = new DirectionCalc();
    SpeedClass SpeedClass = new SpeedClass();

    //variables for the autonomous

    double action = 1;
    boolean oneloop = false;

    double paraSet = 0, perpSet = 0;
    double paraStart = 0, perpStart = 0;
    double speedSet = 15;
    double rampUpDist = 2, rampDownDist = 4;
    double headingSet = 0, headingSpeedSet = 100;
    double liftSet = 0, liftSpeedSet = 1500;
    boolean vuforiatrigger = false;
    double desiredVuforiaAngle = -31;
    double vuforiaOffset = -100000000;
    double waitVariable = 0;
    double intakePower = 0;
    double startOfMatch = 0;
    double loopCycle = 1;



    public static double speedP = .002;
    public static double speedD = .004;

    //AprilTag initialaization here

    OpenCvCamera APRILcamera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;




    @Override

    public void runOpMode() {
        //initializes FTC dahsboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        //initialized the hardware map
        robot.init(hardwareMap);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        robot.AlignmentBar.setPosition(.65);

        telemetry.addLine("initializing apriltags");
        telemetry.update();








        //initilaize the camera for the Apriltags
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        APRILcamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "AprilTagsCamera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        telemetry.addLine("april");
        telemetry.update();

        APRILcamera.setPipeline(aprilTagDetectionPipeline);
        APRILcamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                APRILcamera.startStreaming(1280,720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        telemetry.addLine("apriltags");
        telemetry.update();

        // while init loop for the apriltags
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }
            telemetry.addData("in init", 0);
            telemetry.update();
            sleep(20);
        }


        //start of init
        waitForStart();
        robot.AlignmentBar.setPosition(0.99);

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        //the auto loop
        //ODO.HeadingDEG = vuforiaOffset;
        startOfMatch = getRuntime();
        while (opModeIsActive()) {
            SpeedClass.speedP = speedP;
            SpeedClass.speedD = speedD;

            robot.angles  = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            ODO.OdoCalc(robot.MotorVL.getCurrentPosition(), robot.MotorHL.getCurrentPosition(), robot.MotorVR.getCurrentPosition());

            if(action == 1){
                speedSet = 20;
                rampDownDist = 8;
                rampUpDist = 4;
                liftSet = 825;
                liftSpeedSet = 800;
                paraSet = 18.5;
                perpSet = -1;
                headingSet = -25;
                paraStart = 0;
                perpStart = 0;
                headingSpeedSet = 35;

                if(DirectionCalc.distanceFrom < 3 && oneloop && Math.abs(HDing.headingError) < 20){
                    action = 2;
                    oneloop = false;
                }else{
                    oneloop = true;
                }

            }else if(action == 2){
                paraSet = 35.5;
                perpSet = -3.5;
                headingSet = -30;
                paraStart = 18;
                perpStart = -1;
                headingSpeedSet = 25;

                if(DirectionCalc.distanceFrom < 1 && oneloop && Math.abs(HDing.headingError) < 5){
                    action = 3;
                    oneloop = false;
                }else{
                    oneloop = true;
                }

            } else if(action == 3){
                liftSet = 725;
                if(robot.MotorLift.getCurrentPosition() < 740){
                    action = 4;
                    waitVariable = getRuntime();
                }
            }else if(action == 4){// 1st dropping on mid junction
                intakePower = -.5;
                if(waitVariable + .5 < getRuntime()){
                    action = 5;
                    intakePower = 0;
                }

            }else if(action == 5){
                headingSpeedSet = 50;
                paraSet = 37;
                perpSet = 3;
                headingSet = 0;
                paraStart = 37;
                perpStart = -2.5;
                if(DirectionCalc.distanceFrom < 1 && oneloop && Math.abs(HDing.headingError) < 10){
                    action = 6;
                    oneloop = false;
                }else{
                    oneloop = true;
                }
            }else if(action == 6){
                liftSet = 150;
                paraSet = 54;
                perpSet = 3;
                headingSet = 90;
                paraStart = 36;
                perpStart = 3;
                if(DirectionCalc.distanceFrom < 1 && oneloop){
                    action = 7;
                    oneloop = false;
                }else{
                    oneloop = true;
                }
            }else if(action == 7){
                headingSpeedSet = 50;
                rampDownDist = 5;
                liftSet = 150;
                paraSet = 53.5;
                perpSet = 23;
                headingSet = 90;
                paraStart = 54;
                perpStart = 3;
                if(DirectionCalc.distanceFrom < 2 && oneloop && Math.abs(HDing.headingError) < 10){
                    action = 8;
                    oneloop = false;
                }else{
                    oneloop = true;
                }
            }else if(action == 8){//intaking
                intakePower = .5;
                headingSpeedSet = 75;
                rampDownDist = 10;

                if(loopCycle == 1){
                    liftSet = 150;
                }else if(loopCycle == 2){
                    liftSet = 110;
                }else if(loopCycle == 3){
                    liftSet = 70;
                }else if(loopCycle == 4){
                    liftSet = 40;
                }else{
                    liftSet = 0;
                }

                paraSet = 53.5;
                perpSet = 30;
                headingSet = 85;
                paraStart = 53.5;
                perpStart = 23;
                if(DirectionCalc.distanceFrom < 1 && oneloop && Math.abs(HDing.headingError) < 5 || robot.IntakeDS.getDistance(DistanceUnit.INCH) < 1.5){
                    action = 9;
                    perpSet = ODO.PerpDist - .5;
                    oneloop = false;
                }else{
                    oneloop = true;
                }
            } else if (action == 9) {//lifting lift over cone stack
                ODO.HeadingDEG = -robot.angles.firstAngle;
                ODO.HeadingRAD = Math.toRadians(-robot.angles.firstAngle);
                intakePower = 0;
                headingSpeedSet = 75;
                rampDownDist = 10;
                liftSet = 450;
                paraSet = 53.5;
                headingSet = 85;
                paraStart = 53.5;
                perpStart = 23;
                if(robot.MotorLift.getCurrentPosition() > 425){
                    action = 10;
                    oneloop = false;
                }else{
                    oneloop = true;
                }

            }else if(action == 10) {
                speedSet = 20;
                rampUpDist = 8;
                headingSpeedSet = 180;
                rampDownDist = 5;
                paraSet = 55;
                perpSet = -7.75;
                headingSet = 0;
                paraStart = 52.5;
                perpStart = 29;
                if(DirectionCalc.distanceFrom < 25 && oneloop){
                    liftSet = 1125;
                }
                if (DirectionCalc.distanceFrom < 1 && oneloop && Math.abs(HDing.headingError) < 5) {
                    action = 10.5;
                    oneloop = false;
                } else {
                    oneloop = true;
                }
            }else if(action == 10.5){
                speedSet = 20;
                rampUpDist = 8;
                headingSpeedSet = 180;
                rampDownDist = 5;
                paraSet = 57.75;
                perpSet = -7.75;
                headingSet = 0;
                paraStart = 55;
                perpStart = -7.75;
                if (DirectionCalc.distanceFrom < 1 && oneloop && Math.abs(HDing.headingError) < 5) {
                    action = 11;
                    oneloop = false;
                } else {
                    oneloop = true;
                }
                if(HDing.headingCurrentSpeed < 5){
                    ODO.HeadingDEG = -robot.angles.firstAngle;
                    ODO.HeadingRAD = Math.toRadians(-robot.angles.firstAngle);
                }
            }else if(action == 11){
                liftSet = 1050;
                if(robot.MotorLift.getCurrentPosition() < 1075){
                    action = 12;
                    waitVariable = getRuntime();
                }
            }else if(action == 12){
                intakePower = -.5;
                if(waitVariable + .5 < getRuntime()){
                    action = 12.5;
                    intakePower = 0;
                }
            }else if(action == 12.5){
                speedSet = 20;
                rampUpDist = 8;
                headingSpeedSet = 180;
                rampDownDist = 5;
                paraSet = 55;
                perpSet = -7.75;
                headingSet = 0;
                paraStart = 57.75;
                perpStart = -7.75;
                if (DirectionCalc.distanceFrom < 1 && oneloop && Math.abs(HDing.headingError) < 5) {
                    if(30 + startOfMatch - getRuntime() < 6){
                        action = 15;
                    }else{
                        action = 13;
                        loopCycle = loopCycle + 1;
                    }
                    oneloop = false;
                } else {
                    oneloop = true;
                }
            }
            else if(action == 13){
                if(robot.MotorLift.getCurrentPosition() > 800){
                    speedSet = 15;
                }else{
                    speedSet = 22;
                }
                if(ODO.HeadingDEG > -30){
                    liftSet = 120;
                }
                headingSpeedSet = 75;
                rampDownDist = 10;
                rampUpDist = 4;
                paraSet = 53.5;
                perpSet = 23;
                headingSet = 90;
                paraStart = 55;
                perpStart = -7.75 ;
                if (DirectionCalc.distanceFrom < 1 && oneloop && Math.abs(HDing.headingError) < 5) {
                    if(30 + startOfMatch - getRuntime() < 5){
                        action = 15;
                    }else{
                        action = 8;
                    }

                    oneloop = false;
                } else {
                    oneloop = true;
                }
            }
            else if (action == 15) {
                if(!oneloop){
                    paraStart = paraSet;
                    perpStart = perpSet;
                }
                liftSet = 0;
                paraSet = 54;
                headingSet = 0;

                if (tagOfInterest.id == 1) {
                    perpSet = -18;
                } else if (tagOfInterest.id == 2) {
                    perpSet = 0;
                } else {
                    perpSet = 26;
                }

                if (DirectionCalc.distanceFrom < 1 && oneloop && Math.abs(HDing.headingError) < 5) {
                    action = 16;
                    oneloop = false;
                } else {
                    oneloop = true;

                }
            } else {
                break;
            }

            if(robot.MotorLift.getCurrent(CurrentUnit.MILLIAMPS) > 4000 && liftSet < robot.MotorLift.getCurrentPosition() - 100){
                robot.MotorLift.setPower(0);
                liftSet = robot.MotorLift.getCurrentPosition() + 50;
            }else{
                robot.MotorLift.setPower(Lift.liftpower);
            }

            Lift.LiftMethod(liftSet, liftSpeedSet, robot.MotorLift.getCurrentPosition(), getRuntime());
            Drivetrain(paraSet, perpSet, paraStart, perpStart, speedSet, rampUpDist, rampDownDist, headingSet, headingSpeedSet, ODO.ParaDist, ODO.PerpDist, ODO.HeadingDEG, getRuntime());
            robot.MotorLift.setPower(Lift.liftpower);
            robot.IntakeS.setPower(intakePower);
            robot.MotorVL.setPower(LLDIR);
            robot.MotorVR.setPower(LRDIR);
            robot.MotorHL.setPower(RLDIR);
            robot.MotorHR.setPower(RRDIR);

            telemetry.addData("lift", robot.MotorLift.getCurrentPosition());
            telemetry.addData("robot speed", SpeedClass.currentSpeed);
            telemetry.addData("heading error abs", Math.abs(HDing.headingError));
            telemetry.addData("heading power", HDing.headingPower);
            telemetry.addData("FinalX", FinalX);
            telemetry.addData("FinalY", FinalY);
            telemetry.addData("direction vector", DirectionCalc.directionVector);
            telemetry.addData("slope", DirectionCalc.slope);
            telemetry.addData("speed mpower", SpeedClass.speedPower);
            telemetry.addData("dist from", DirectionCalc.distanceFrom);
            telemetry.addData("para current", ODO.ParaDist);
            telemetry.addData("perp current", ODO.PerpDist);
            telemetry.addData("heading", ODO.HeadingDEG);
            telemetry.addData("para dist from", DirectionCalc.paradistfrom);
            telemetry.addData("action", action);
            telemetry.addData("para dist", ODO.ParaDist);
            telemetry.addData("perp dist", ODO.PerpDist);
            telemetry.addData("hasdistfrom", DirectionCalc.hasDistFrom);
            telemetry.addData("Total dist", DirectionCalc.totalDist);
            telemetry.addData("LLDIR", LLDIR);
            telemetry.addData("max motor", MaxMotor);
            telemetry.addData("motor speed", MotorSpeed);

            telemetry.update();

            dashboardTelemetry.addData("direction vector", DirectionCalc.directionVector);
            dashboardTelemetry.addData("distance from", DirectionCalc.distanceFrom);
            dashboardTelemetry.addData("setX", DirectionCalc.setX);
            dashboardTelemetry.addData("setY", DirectionCalc.setY);
            dashboardTelemetry.addData("current Para", ODO.ParaDist);
            dashboardTelemetry.addData("current Perp", ODO.PerpDist);
            dashboardTelemetry.addData("speed", SpeedClass.currentSpeed);
            dashboardTelemetry.addData("speed Set", SpeedClass.speedSetMod);
            dashboardTelemetry.addData("speedpower", SpeedClass.speedPower);
            dashboardTelemetry.addData("combined motor Speed", MotorSpeed);
            dashboardTelemetry.addData("direction vector", DirectionCalc.directionVector);

            dashboardTelemetry.update();


        }


    }
    public double FinalX = 0;
    public double FinalY = 0;

    public double RLDIR, RRDIR, LRDIR, LLDIR;

    double MaxMotor = 0;

    double MotorSpeed = 0;




    public void Drivetrain(double paraSet, double perpSet, double parastart, double perpstart, double speedSet, double rampUpDist ,double rampDownDist, double headingSet,
                           double headingSpeed, double currentpara, double currentperp, double currentheading, double time){

        DirectionCalc.DirectionMethod(paraSet, perpSet, parastart, perpstart, currentpara, currentperp);
        SpeedClass.SpeedCalc(speedSet, rampUpDist,rampDownDist, DirectionCalc.distanceFrom, currentpara, currentperp, time);
        HDing.HeadingMethod(headingSet, headingSpeed, currentheading, time);


        FinalY = 1 * Math.cos(Math.toRadians(DirectionCalc.directionVector - ODO.HeadingDEG));
        FinalX = 1 * Math.sin(Math.toRadians(DirectionCalc.directionVector - ODO.HeadingDEG));

        LLDIR = FinalY + HDing.headingPower;
        LRDIR = -FinalY + HDing.headingPower;
        RLDIR = -FinalX + HDing.headingPower;
        RRDIR = FinalX + HDing.headingPower;

        MaxMotor = Math.max(Math.max(Math.abs(LLDIR), Math.abs(LRDIR)), Math.max(Math.abs(RLDIR), Math.abs(RRDIR)));

        LLDIR = LLDIR/MaxMotor;
        LRDIR = LRDIR/MaxMotor;
        RLDIR = RLDIR/MaxMotor;
        RRDIR = RRDIR/MaxMotor;

        if(DirectionCalc.distanceFrom < .5){
            SpeedClass.speedPower = 0;
        }
        if(Math.abs(HDing.headingError) < 3){
            HDing.headingPower = 0;
        }

        MotorSpeed = Math.abs(SpeedClass.speedPower) + Math.abs(HDing.headingPower);

        if(MotorSpeed > 1){
            MotorSpeed = 1;
        }else if(MotorSpeed < -1){
            MotorSpeed = -1;
        }

        if(DirectionCalc.distanceFrom < .5 && Math.abs(HDing.headingError) < 2){
            HDing.headingPower = 0;
            SpeedClass.speedPower = 0;
            LLDIR = 0;
            LRDIR = 0;
            RLDIR = 0;
            RRDIR = 0;
        }else{
            LLDIR = LLDIR * MotorSpeed;
            LRDIR = LRDIR * MotorSpeed;
            RLDIR = RLDIR * MotorSpeed;
            RRDIR = RRDIR * MotorSpeed;
        }






    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }


}


