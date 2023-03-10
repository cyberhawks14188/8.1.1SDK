package org.firstinspires.ftc.teamcode;


import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.DriveCode.DirectionCalc;
import org.firstinspires.ftc.teamcode.DriveCode.HeadingControl;
import org.firstinspires.ftc.teamcode.DriveCode.OdometryCode;
import org.firstinspires.ftc.teamcode.DriveCode.Smoothing;
import org.firstinspires.ftc.teamcode.DriveCode.SpeedClass;
import org.firstinspires.ftc.teamcode.LiftClasses.LiftControl;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Config
@Autonomous

public class RightPlaceODOAuto extends LinearOpMode {

    Jake_2_Hardware robot = new Jake_2_Hardware();
    OdometryCode ODO = new OdometryCode();
    LiftControl Lift = new LiftControl();
    HeadingControl HDing = new HeadingControl();
    Smoothing Smoothing = new Smoothing();
    DirectionCalc DirectionCalc = new DirectionCalc();
    SpeedClass SpeedClass = new SpeedClass();
    openCVTestign openCV = new openCVTestign();


    //variables for the autonomous

    double action = 1;
    boolean oneloop = false;

    double paraSet = 0, perpSet = 0;
    double paraStart = 0, perpStart = 0;
    double speedSet = 15;
    double rampUpDist = 2, rampDownDist = 4;
    double headingSet = 0, headingSpeedSet = 100;
    double liftSet = 0, liftSpeedSet = 1500;
    double waitVariable = 0;
    double intakePower = 0;
    double startOfMatch = 0;
    double loopCycle = 1;
    double lastHeadingSet = 0;
    boolean FieldCentricTrigger = true;
    boolean resetTrigger = false;
    double minDist = 10000;
    double minDistPara = 0, minDistPerp = 0;
    boolean hasDistanceFrom = false;
    double intakeStackNum = 1;
    double lastAction = 0;
    double intakedPara = 0;
    double intakedPerp = 0;
    double IMUTimer = 10000;
    double lowJunctionPerp = 0;
    double tapeCenterError = 0,  tapecenterLastError = 0;
    boolean zeroZoneTrigger = true;

    public static double speedP = .002;
    public static double speedD = .004;
    public static double cameraP = .000023;//originally -.01
    public static double cameraD = 0.00008;

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
    OpenCvWebcam webcam;



    @SuppressLint("DefaultLocale")
    @Override

    public void runOpMode() {
        //initializes FTC dahsboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        openCV.redBlueTrigger = false;
        openCV.colorTrigger = false;


        //initialized the hardware map
        robot.init(hardwareMap);


        robot.AlignmentBar.setPosition(.65);


        telemetry.addLine("initializing apriltags");
        telemetry.update();


        //initilaize the camera for the Apriltags
       // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        APRILcamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "AprilTagsCamera"));//, cameraMonitorViewId);
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
                FtcDashboard.getInstance().startCameraStream(APRILcamera, 30);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });


       // webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "IntakeCamera"), cameraMonitorViewId);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "IntakeCamera"));

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "IntakeCamera"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        webcam.setPipeline(new openCVTestign.BlueOpenCV_Pipeline());

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        //webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);



            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        //telemetry.setMsTransmissionInterval(50);

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
        APRILcamera.closeCameraDevice();

        robot.MotorVL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorVR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorHL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorHR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.MotorVL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.MotorVR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.MotorHL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.MotorHR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ODO.HeadingDEG = 0;
        ODO.HeadingRAD = Math.toRadians(0);
        ODO.ParaDist = 0;
        ODO.PerpDist = 0;


        //the auto loop
        startOfMatch = getRuntime();
        robot.AlignmentBar.setPosition(0.99);

        startOfMatch = getRuntime();
        while (opModeIsActive()) {

            if(getRuntime() - startOfMatch > 25){
                action = 99;
                oneloop = false;
            }

            robot.angles  = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            SpeedClass.speedP = speedP;
            SpeedClass.speedD = speedD;

            ODO.OdoCalc(robot.MotorVL.getCurrentPosition(), robot.MotorHL.getCurrentPosition(), robot.MotorVR.getCurrentPosition());

            if(action == 1) {//move 1 to aim to the alliance High junction
                speedSet = 23;
                rampDownDist = 8;
                liftSpeedSet = 1200;
                liftSet = 200;
                paraSet = 10;
                perpSet = -19;
                paraStart = 0;
                perpStart = 0;
                headingSpeedSet = 130;
                headingSet = -90;
                if (DirectionCalc.distanceFrom < 2 && oneloop && Math.abs(HDing.headingError) < 10) {
                    action = 2;
                    oneloop = false;
                } else {
                    oneloop = true;
                }

            }else if(action == 2) {//move 1 to aim to the alliance High junction
                speedSet = 20;
                rampDownDist = 5;
                paraSet = 43;
                perpSet = -25;
                paraStart = 10;
                perpStart = -19;
                headingSpeedSet = 130;
                headingSet = -90;
                if(ODO.ParaDist > 18){
                    liftSet = 1125;
                }

                if (DirectionCalc.distanceFrom < 2 && oneloop && Math.abs(HDing.headingError) < 10) {
                    action = 3;
                    oneloop = false;
                } else {
                    oneloop = true;
                }

            }else if(action == 3){
                if(!oneloop){
                    waitVariable = getRuntime() + .75;
                    oneloop = true;
                }

                if(waitVariable != 0 && waitVariable < getRuntime()){
                    action = 4;
                    oneloop = false;
                    liftSet = 1000;
                    waitVariable = 0;

                }
            }else if(action == 4){
                if(robot.MotorLift.getCurrentPosition() < 1050 && waitVariable == 0){
                    waitVariable = getRuntime() + .5;
                    intakePower = -.5;
                }
                if(waitVariable != 0 && waitVariable < getRuntime() && intakePower == -.5){
                    liftSet = 1125;
                    if(robot.MotorLift.getCurrentPosition() > 1100){
                        action = 5;
                        waitVariable = 0;
                        intakePower = 0;
                    }
                }

            }else if(action == 5){//moves out of the way of the high pole
                speedSet = 20;
                paraSet = 52;
                perpSet = -18;
                paraStart = 43;
                perpStart = -25;
                headingSpeedSet = 100;
                headingSet = -270;
                if(ODO.HeadingDEG < -110){
                    liftSet = 200;
                }
                if (DirectionCalc.distanceFrom < 4 && oneloop) {
                    action = 6;
                    oneloop = false;
                } else {
                    oneloop = true;
                }

            } else if(action == 6){//gets close to the blue tape
                rampDownDist = 4;
                paraSet = 53;
                perpSet = 21;
                paraStart = 52;
                perpStart = -18;
                headingSpeedSet = 200;
                headingSet = -270;

                if (DirectionCalc.distanceFrom < 2 && oneloop && Math.abs(HDing.headingError) < 10) {
                    action = 7;
                    liftSet = 200;
                    oneloop = false;
                    speedSet = 15;
                    if(robot.angles.firstAngle > 0){
                        ODO.HeadingDEG = 0 - robot.angles.firstAngle;
                        ODO.HeadingRAD = Math.toRadians(0 - robot.angles.firstAngle);
                    }else{
                        ODO.HeadingDEG = -180 - (180 - Math.abs(robot.angles.firstAngle));
                        ODO.HeadingRAD = Math.toRadians(-180 - (180 - Math.abs(robot.angles.firstAngle)));
                    }

                } else {
                    oneloop = true;
                }

            }else if(action == 7){//centers on the tape and goes to intake
                intakePower = .5;
                if(intakeStackNum == 1){
                    liftSet = 175;
                }else if(intakeStackNum == 2){
                    liftSet = 125;
                }else if(intakeStackNum == 3){
                    liftSet = 80;
                }else if(intakeStackNum == 4){
                    liftSet = 40;
                }else {
                    liftSet = 0;
                }

                zeroZoneTrigger = false;
                rampDownDist = .000001;
                tapecenterLastError = tapeCenterError;

                tapeCenterError = (robot.IntakeLeftColor.blue() - robot.IntakeRightColor.blue());

                if(Math.abs(tapeCenterError) > 1000){
                    paraSet = paraSet - (-cameraP * tapeCenterError) + ((tapeCenterError - tapecenterLastError) * -cameraD);
                }



                speedSet = 8;
                perpSet = 25;
                paraStart = 53;
                perpStart = -18;
                headingSpeedSet = 200;
                headingSet = -270;

                if(robot.IntakeDS.getDistance(DistanceUnit.INCH) > 2){
                    perpSet = ODO.PerpDist + 2;
                }else{
                    perpSet = ODO.PerpDist;
                    perpStart = ODO.PerpDist;
                    paraStart = ODO.ParaDist;
                    intakedPara = ODO.ParaDist;
                    minDist = 1000;

                    action = 8;
                    webcam.setPipeline(new openCVTestign.OpenCV_Pipeline());
                    intakeStackNum = intakeStackNum + 1;
                }


            }else if(action == 8){//aligns with mid junction
                liftSet = 825;
                if(robot.MotorLift.getCurrentPosition() > 300){
                    rampDownDist = 4;
                    speedSet = 20;
                    paraSet = intakedPara - 6;
                    perpSet = -10;
                    headingSpeedSet = 150;
                    headingSet = -180;
                    /*   if(ODO.HeadingDEG > -190 && robot.MotorLift.getCurrentPosition() > 100 && ODO.PerpDist > 0){
                        if(robot.BaseDS.getDistance(DistanceUnit.INCH) < minDist){
                            minDist = robot.BaseDS.getDistance(DistanceUnit.INCH);
                            lowJunctionPerp = ODO.PerpDist;
                        }
                    }*/

                    if (DirectionCalc.distanceFrom < 1 && oneloop && Math.abs(HDing.headingError) < 5) {
                        action = 9;
                        oneloop = false;
                    } else {
                        oneloop = true;
                    }

                }
            }else if(action == 9){//lets robot settle



           /*     if(Math.abs(320 - openCVTestign.poleCenterX) < 65){
                    perpSet = Math.copySign(ODO.PerpDist - (cameraP * ((320 - openCVTestign.poleCenterX))/65), ODO.PerpDist + (cameraP * (320 - openCVTestign.poleCenterX)));
                }else{
                    perpSet = ODO.PerpDist - (cameraP * (320 - openCVTestign.poleCenterX));
                }*/

                if(HDing.headingCurrentSpeed < 5){
                    if(robot.angles.firstAngle > 0){
                        ODO.HeadingDEG = 0 - robot.angles.firstAngle;
                        ODO.HeadingRAD = Math.toRadians(0 - robot.angles.firstAngle);
                    }else{
                        ODO.HeadingDEG = -180 - (180 - Math.abs(robot.angles.firstAngle));
                        ODO.HeadingRAD = Math.toRadians(-180 - (180 - Math.abs(robot.angles.firstAngle)));
                    }
                }

                if(!oneloop){
                    waitVariable = getRuntime() + .75;
                    oneloop = true;
                }

                if(waitVariable != 0 && waitVariable < getRuntime()){
                    action = 10;
                    oneloop = false;
                    liftSet = 750;
                    waitVariable = 0;
                    webcam.setPipeline(new openCVTestign.BlueOpenCV_Pipeline());
                }



            }else if(action == 10){//drops cone
                if(robot.MotorLift.getCurrentPosition() < 1050 && waitVariable == 0){
                    waitVariable = getRuntime() + .5;
                    intakePower = -.5;

                }
                if(waitVariable != 0 && waitVariable < getRuntime() && intakePower == -.5){
                    liftSet = 825;
                    if(robot.MotorLift.getCurrentPosition() > 800){
                        action = 11;
                        oneloop = false;
                        waitVariable = 0;
                        intakePower = 0;
                    }
                }
            }else if(action == 11){//goes back to the stack
                rampDownDist = 4;
                if(intakedPara != 0){
                    paraSet = intakedPara;
                }else{
                    paraSet = 52;
                }
                perpSet = 21;
                paraStart = 52;
                perpStart = -18;
                headingSpeedSet = 30;
                speedSet = 10;
                headingSet = -270;

                if(ODO.HeadingDEG < -200){
                    liftSet = 200;
                    speedSet = 15;
                    headingSpeedSet = 90;
                }

                if (DirectionCalc.distanceFrom < 2 && oneloop && Math.abs(HDing.headingError) < 10) {
                    action = 7;
                    oneloop = false;
                    speedSet = 15;
                } else {
                    oneloop = true;
                }
            }else if(action == 99){
                if(!oneloop){
                    perpStart = ODO.PerpDist;
                    paraStart = ODO.ParaDist;
                }

                speedSet = 23;
                rampDownDist = 4;
                liftSpeedSet = 1200;
                liftSet = 200;

                headingSpeedSet = 130;
                headingSet = 0;
                paraSet = 52;
                if(tagOfInterest.id == 1){
                    perpSet = -20;
                }else if(tagOfInterest.id == 2){
                    perpSet = 2;
                }else{
                    perpSet = 26;
                }

                if (DirectionCalc.distanceFrom < 2 && oneloop && Math.abs(HDing.headingError) < 10) {
                    action = 100;
                    oneloop = false;
                } else {
                    oneloop = true;
                }
            }else {
                break;
            }

            if(SpeedClass.currentSpeed < 1 && HDing.headingCurrentSpeed < 5 && IMUTimer == 10000){
                IMUTimer = getRuntime() + .5;
            }else{
                IMUTimer = 10000;
            }

         /*   if(IMUTimer < getRuntime()){
                if(robot.angles.firstAngle > 0){
                    ODO.HeadingDEG = 0 - robot.angles.firstAngle;
                    ODO.HeadingRAD = Math.toRadians(0 - robot.angles.firstAngle);
                }else{
                    ODO.HeadingDEG = -180 - (180 - Math.abs(robot.angles.firstAngle));
                    ODO.HeadingRAD = Math.toRadians(-180 - (180 - Math.abs(robot.angles.firstAngle)));
                }
                IMUTimer = 10000;
            }

          */

            lastAction = action;

            Lift.LiftMethod(liftSet, liftSpeedSet, robot.MotorLift.getCurrentPosition(), getRuntime());
            Drivetrain(paraSet, perpSet, paraStart, perpStart, speedSet, rampUpDist, rampDownDist, headingSet, headingSpeedSet, ODO.ParaDist, ODO.PerpDist, ODO.HeadingDEG, getRuntime());
            robot.MotorLift.setPower(Lift.liftpower);
            robot.IntakeS.setPower(intakePower);
            robot.MotorVL.setPower(LLDIR);
            robot.MotorVR.setPower(LRDIR);
            robot.MotorHL.setPower(RLDIR);
            robot.MotorHR.setPower(RRDIR);


            telemetry.addData("angles.first", robot.angles.firstAngle);
            telemetry.addData("intaked Para", intakedPara);
            telemetry.addData("lowjunctionperp", lowJunctionPerp);
            telemetry.addData("base DS", robot.BaseDS.getDistance(DistanceUnit.INCH));
            telemetry.addData("test", Math.abs(320 - openCVTestign.poleCenterX));

            telemetry.addData("lift", robot.MotorLift.getCurrentPosition());
            telemetry.addData("heading error abs", Math.abs(HDing.headingError));
            telemetry.addData("heading power", HDing.headingPower);
            telemetry.addData("speed power", SpeedClass.speedPower);
            telemetry.addData("dist from", DirectionCalc.distanceFrom);
            telemetry.addData("para current", ODO.ParaDist);
            telemetry.addData("perp current", ODO.PerpDist);

            telemetry.addData("heading", ODO.HeadingDEG);
            telemetry.addData("action", action);
            telemetry.update();
            dashboardTelemetry.addData("tapeCenterError", tapeCenterError);
            dashboardTelemetry.addData("robot.IntakeLeftColor.blue()", robot.IntakeLeftColor.blue());
            dashboardTelemetry.addData("robot.IntakeRightColor.blue()", robot.IntakeRightColor.blue());
            dashboardTelemetry.update();

            lastHeadingSet = headingSet;


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

        if(zeroZoneTrigger){
            if(DirectionCalc.distanceFrom < .375){
                SpeedClass.speedPower = 0;
                SpeedClass.lastSpeedError = 0;
            }
        }

        if(Math.abs(HDing.headingError) < .5){
            HDing.headingPower = 0;
        }

        MotorSpeed = Math.abs(SpeedClass.speedPower) + Math.abs(HDing.headingPower);

        if(MotorSpeed > 1){
            MotorSpeed = 1;
        }else if(MotorSpeed < -1){
            MotorSpeed = -1;
        }

      /*  if(DirectionCalc.distanceFrom < .5 && Math.abs(HDing.headingError) < 2){
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
        }*/

        LLDIR = LLDIR * MotorSpeed;
        LRDIR = LRDIR * MotorSpeed;
        RLDIR = RLDIR * MotorSpeed;
        RRDIR = RRDIR * MotorSpeed;






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


