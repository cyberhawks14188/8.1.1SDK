package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.DriveCode.HeadingControl;
import org.firstinspires.ftc.teamcode.DriveCode.OdometryCode;
import org.firstinspires.ftc.teamcode.DriveCode.Smoothing;
import org.firstinspires.ftc.teamcode.DriveCode.SpeedClass;
import org.firstinspires.ftc.teamcode.LiftClasses.LiftControl;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class Expiremental_Jake_2_TeleOp extends LinearOpMode {
    //test
    public static double tickstoin = 1825;
    public static double Trackwidth = 8.35;
    public static double VertOffset = 4.75;
    public static double zP = .0006;
    public static double zD = 0.002;
    public static double LIFTP = .0003;
    public static double LIFTSPEEDSET = 1500;
    public static double CAMERAP = .1;

    double x, y, z;
    double finalX, finalY;
    double RLDIR, RRDIR, LRDIR, LLDIR;
    double liftSpeedSet = 1500;
    double lastIntakeSensor = 10;
    boolean wasIntakeOpen = true;

    double vectorMagnitude = 0, vectorAngleRAD = 0, vectorAngleDEG = 0;
    double finalvectorAngleDEG = 0;

    double IMU = 0;

    double liftset = 0;
    double liftcurrentpos;

    double drivespeed = 1;

    double headingsetpoint = 0;

    double Maxpower = 0;

    boolean lastx = false;
    boolean lasty = false;

    double alignmentBarSet = 0.99;
    boolean lastStart = false;
    double IMUSpeedCurrent = 0;
    boolean IMUSpeedTrigger = false;
    double lastIMUSpeed = 0;
    double lastTime = 0;
    boolean lastIMUSpeedTrigger = false;
    double IMUTimer = 10000;




    Jake_2_Hardware robot = new Jake_2_Hardware();
    OdometryCode ODO = new OdometryCode();
    LiftControl lift = new LiftControl();
    HeadingControl HDing = new HeadingControl();
    Smoothing Smoothing = new Smoothing();
    SpeedClass SpeedClass = new SpeedClass();
    openCVTestign openCV = new openCVTestign();
    OpenCvWebcam webcam;


    @Override

    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();



        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "IntakeCamera"), cameraMonitorViewId);
        webcam.setPipeline(new openCVTestign.OpenCV_Pipeline());
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                FtcDashboard.getInstance().startCameraStream(webcam, 30);
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
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);

            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();



        while (opModeIsActive()) {
            //****** Basic variable setting for correct robot code ***************
            HDing.inTeleOp = true;
            SpeedClass.SpeedCalc(1,0, 0, 0, ODO.ParaDist, ODO.PerpDist, getRuntime());




            //********* FTC DashBoard Stuff*************
            lift.liftP = LIFTP;
            ODO.PerpOffset = VertOffset;
            ODO.TicksToInches = tickstoin;
            ODO.TrackWidth = Trackwidth;
            HDing.headingP = zP;
            HDing.headingD = zD;




            //*******IMU RESET ANGLE*******************

           // Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            IMUSpeedCurrent = (IMUSpeedCurrent - lastIMUSpeed)/(getRuntime() - lastTime);

            if(IMUSpeedCurrent < 5){
                IMUSpeedTrigger = true;
                if(IMUSpeedTrigger && !lastIMUSpeedTrigger){
                    IMUTimer = getRuntime() + .5;
                }
            }else{
                IMUSpeedTrigger = false;
                IMUTimer = getRuntime() + 100000;
            }
/*
            if(IMUTimer < getRuntime()){
                ODO.HeadingDEG = angles.firstAngle;
                headingsetpoint = angles.firstAngle;
            }

            lastIMUSpeedTrigger = IMUSpeedTrigger;


*/


            //******** Manual Heading Reset *******************

            if( gamepad1.back){
                ODO.HeadingRAD = Math.toRadians(0);
                headingsetpoint = 0;
            }


            //*********Odometry Calculation*************
            ODO.OdoCalc(robot.MotorVL.getCurrentPosition(), robot.MotorHL.getCurrentPosition(), robot.MotorVR.getCurrentPosition());



            //**********Alignemnt Bar Positioning************
            if(gamepad1.start && !lastStart){
                lastStart = true;
                if(alignmentBarSet != .65){
                    alignmentBarSet = .65;
                }else{
                    alignmentBarSet = .99;
                }
            }

            if(!gamepad1.start){
                lastStart = false;
            }

            robot.AlignmentBar.setPosition(alignmentBarSet);





            //************ Lift Code ****************
            liftcurrentpos = -robot.MotorLift.getCurrentPosition();

            if(gamepad1.dpad_down){
                liftset = 0;
            }else if(gamepad1.dpad_left){
                liftset = 500;
            }else if(gamepad1.dpad_right){
                liftset = 825;
            }else if(gamepad1.dpad_up) {
                liftset = 1125;
            }else{
                if(gamepad1.left_bumper){
                    liftset += 15 * -gamepad1.right_stick_y;
                }
            }

           /* if(wasIntakeOpen == true && robot.IntakeV3.getDistance(DistanceUnit.INCH) < 1.5){
                liftset = liftset + 50;
                wasIntakeOpen = false;
            }

            if(robot.IntakeV3.getDistance(DistanceUnit.INCH) > 1.5){
                wasIntakeOpen = true;
            }*/


            if(gamepad1.x && lastx == false){
                liftset = liftset - 75;
            }

            if(gamepad1.x){
                lastx = true;
            }else{
                lastx = false;
            }

            if(gamepad1.y && lasty == false){
                liftset = liftset + 75;
            }

            if(gamepad1.y){
                lasty = true;
            }else{
                lasty = false;
            }

            if(liftset < liftcurrentpos){
                liftSpeedSet = 800;
            }else{
                liftSpeedSet = 1500;
            }

            lift.LiftMethod(liftset, liftSpeedSet, liftcurrentpos, getRuntime());


            if(robot.MotorLift.getCurrent(CurrentUnit.MILLIAMPS) > 4000 && liftset < liftcurrentpos - 100){
                robot.MotorLift.setPower(0);
                liftset = liftcurrentpos + 50;
            }else{
                robot.MotorLift.setPower(lift.liftpower);
            }


            //*********** Intake Code ***************

            if(gamepad1.a){
                robot.IntakeS.setPower(.5);
            }else if(gamepad1.b){
                robot.IntakeS.setPower(-.5);
            }else{
                robot.IntakeS.setPower(0.2);
            }


            //*********** Drive Code ************************

            x = Smoothing.SmoothPerpendicularInput(Math.copySign( gamepad1.left_stick_x, gamepad1.left_stick_x));
            y = -Smoothing.SmoothParallelInput(Math.copySign( gamepad1.left_stick_y, gamepad1.left_stick_y));
            if (vectorAngleDEG < 0) {
                vectorAngleDEG = vectorAngleDEG + 360;
            }

            //defines the vector
            //angle should output 0-360 deg
            vectorMagnitude = Math.sqrt((y * y) + (x * x));//hypotenuse of the joysticks
            vectorAngleRAD = Math.atan2(y, x);//angle of the joystick inputs in Radians
            vectorAngleDEG = -Math.toDegrees(vectorAngleRAD);//converts that into degrees

            //Subtracts the heading angle to create the virtual forward
            finalvectorAngleDEG = vectorAngleDEG - ODO.HeadingDEG;

            //convert back into x and y values



            if(gamepad1.left_trigger > .05){
               /* if(robot.BaseDistanceSensor.getDistance(DistanceUnit.INCH) > 5){
                    headingsetpoint = ODO.HeadingDEG + (-.1 * (320 - openCV.poleCenterX));
                    finalX = 0;
                    drivespeed = .3;

                    finalY += -CAMERAP * (5 - robot.BaseDistanceSensor.getDistance(DistanceUnit.INCH));
                }else{
                    drivespeed = 0;
                }*/




            }else{
                finalX = vectorMagnitude * Math.cos(Math.toRadians(finalvectorAngleDEG));
                finalY = vectorMagnitude * Math.sin(Math.toRadians(finalvectorAngleDEG));
                if(gamepad1.right_bumper){
                    drivespeed = .4;
                    if(!gamepad1.left_bumper){
                        headingsetpoint += gamepad1.right_stick_x * 2;
                    }
                }else if(gamepad1.right_trigger > .1) {
                    drivespeed = 1;
                    if(!gamepad1.left_bumper){
                        headingsetpoint += gamepad1.right_stick_x * 10;
                    }
                }else{
                    drivespeed = .7;
                    if(!gamepad1.left_bumper){
                        headingsetpoint += gamepad1.right_stick_x * 8;
                    }
                }
            }


            HDing.HeadingMethod(headingsetpoint, 300, ODO.HeadingDEG, getRuntime());

            z = HDing.headingPower;
            if(z > 1){
                z = 1;
            }else if (z < -1){
                z = -1;
            }

            LLDIR = -finalY + z;
            LRDIR = finalY + z;
            RLDIR = -finalX + z;
            RRDIR = finalX + z;

            robot.MotorVL.setPower(LLDIR * drivespeed);
            robot.MotorVR.setPower(LRDIR * drivespeed);
            robot.MotorHL.setPower(RLDIR * drivespeed);
            robot.MotorHR.setPower(RRDIR * drivespeed);

            //***************** Time for lift calc *************
            lastTime = getRuntime();


            //**************** Telemetry **********************
           // dashboardTelemetry.addData("baseDistance", robot.BaseDistanceSensor.getDistance(DistanceUnit.INCH));
            dashboardTelemetry.addData("pole center X", openCV.poleCenterX);
            dashboardTelemetry.addData("pode width", openCV.poleWidth);
            dashboardTelemetry.addData("intake Distance Sensor", robot.IntakeDS.getDistance(DistanceUnit.INCH));
            dashboardTelemetry.addData("heading", ODO.HeadingDEG);
            dashboardTelemetry.addData("headingSet", headingsetpoint);
            dashboardTelemetry.addData("heading Speed", HDing.headingCurrentSpeed);
            dashboardTelemetry.update();
            telemetry.addData("telemetry trasmission speed", telemetry.getMsTransmissionInterval());
            telemetry.addData("lift milliamp", robot.MotorLift.getCurrent(CurrentUnit.MILLIAMPS));
            //telemetry.addData("imu first angle", angles.firstAngle);
            telemetry.addData("aligment bar" ,alignmentBarSet);
            telemetry.addData("right encoder RAW", robot.MotorVR.getCurrentPosition());
            telemetry.addData("Left encoder RAW", robot.MotorVL.getCurrentPosition());
            telemetry.addData("back encoder RAW", robot.MotorHL.getCurrentPosition());

            telemetry.addData("speed", SpeedClass.currentSpeed);
            telemetry.addData("heading current speed", HDing.headingCurrentSpeed);
            telemetry.addData("inmotionprofile?", HDing.inmotionprofileheading);
            telemetry.addData("heading set", headingsetpoint);
            telemetry.addData("heading", ODO.HeadingDEG);
            telemetry.addData("heading power", HDing.headingPower);
            telemetry.addData("para in", ODO.ParaDist);
            telemetry.addData("perp in", ODO.PerpDist);
            telemetry.addData("Encoder Para Left", robot.MotorVL.getCurrentPosition());
            telemetry.addData("Encoder Para Right", robot.MotorVR.getCurrentPosition());
            telemetry.addData("Encoder Perp", robot.MotorHL.getCurrentPosition());
            dashboardTelemetry.addData("liftpos", liftcurrentpos);
            dashboardTelemetry.addData("lift speed", lift.liftSpeed);
            dashboardTelemetry.addData("lift set", liftset);
            dashboardTelemetry.addData("lift power", lift.liftpower);

            telemetry.addData("IMU v", IMU);
            telemetry.addData("liftset", liftset);
            telemetry.addData("lift power", lift.liftpower);
            telemetry.addData("liftpos", liftcurrentpos);


            telemetry.update();


        }
    }
}
