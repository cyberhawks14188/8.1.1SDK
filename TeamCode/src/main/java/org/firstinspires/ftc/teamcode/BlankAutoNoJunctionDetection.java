package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

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

public class BlankAutoNoJunctionDetection extends LinearOpMode {

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
    double waitVariable = 0;
    double intakePower = 0;
    double startOfMatch = 0;
    double loopCycle = 1;
    double lastHeadingSet = 0;
    boolean FieldCentricTrigger = true;



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
    OpenCvWebcam webcam;



    @Override

    public void runOpMode() {
        //initializes FTC dahsboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        //initialized the hardware map
        robot.init(hardwareMap);


        robot.AlignmentBar.setPosition(.65);


        telemetry.addLine("initializing apriltags");
        telemetry.update();


        //initilaize the camera for the Apriltags
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        APRILcamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"));//, cameraMonitorViewId);
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


        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        webcam.setPipeline(new openCVTestign.OpenCV_Pipeline());

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
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

        webcam.setPipeline(new openCVTestign.OpenCV_Pipeline());



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




        //the auto loop
        startOfMatch = getRuntime();
        robot.AlignmentBar.setPosition(0);
        while (opModeIsActive()) {







            SpeedClass.speedP = speedP;
            SpeedClass.speedD = speedD;


            ODO.OdoCalc(robot.MotorVL.getCurrentPosition(), robot.MotorHL.getCurrentPosition(), robot.MotorVR.getCurrentPosition());

            if(action == 1) {//move 1 to aim to the alliance High junction
                speedSet = 20;
                rampDownDist = 4;
                liftSpeedSet = 1200;
                paraSet = 10;
                perpSet = 24;
                paraStart = 0;
                perpStart = 0;
                headingSpeedSet = 90;
                headingSet = -35;
                if (DirectionCalc.distanceFrom < 8) {
                    liftSet = 825;
                } else {
                    liftSet = 0;
                }
                if (DirectionCalc.distanceFrom < 2 && oneloop && Math.abs(HDing.headingError) < 10) {
                    action = 2;
                    oneloop = false;
                } else {
                    oneloop = true;
                }
            }else if(action == 2) {//move 1 to aim to the alliance High junction
                paraSet = 30;
                perpSet = 24;
                paraStart = 10;
                perpStart = 24;
                headingSpeedSet = 90;
                headingSet = -35;
                if (DirectionCalc.distanceFrom < 2 && oneloop && Math.abs(HDing.headingError) < 10) {
                    action = 3;
                    oneloop = false;
                } else {
                    oneloop = true;
                }

            }else if(action == 3){


            } else {
                break;
            }

            Lift.LiftMethod(liftSet, liftSpeedSet, -robot.MotorLift.getCurrentPosition(), getRuntime());
            Drivetrain(paraSet, perpSet, paraStart, perpStart, speedSet, rampUpDist, rampDownDist, headingSet, headingSpeedSet, ODO.ParaDist, ODO.PerpDist, ODO.HeadingDEG, getRuntime());
            robot.MotorLift.setPower(Lift.liftpower);
            robot.IntakeS.setPower(intakePower);
            robot.MotorVL.setPower(LLDIR);
            robot.MotorVR.setPower(LRDIR);
            robot.MotorHL.setPower(RLDIR);
            robot.MotorHR.setPower(RRDIR);

            telemetry.addData("lift", -robot.MotorLift.getCurrentPosition());
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
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());


            telemetry.update();
            //dashboardTelemetry.addData("boudning box x", openCVTestign.OpenCV_Pipeline.poleBoundingBox)
            dashboardTelemetry.addData("headingset", headingSet);
            dashboardTelemetry.addData("headingmod", openCVTestign.poleCenterX);
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


        if(FieldCentricTrigger){
            FinalY = 1 * Math.cos(Math.toRadians(DirectionCalc.directionVector - ODO.HeadingDEG));
            FinalX = 1 * Math.sin(Math.toRadians(DirectionCalc.directionVector - ODO.HeadingDEG));
        }else{
            FinalY = 1 * Math.cos(Math.toRadians(DirectionCalc.directionVector));
            FinalX = 1 * Math.sin(Math.toRadians(DirectionCalc.directionVector));
        }


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
            SpeedClass.lastSpeedError = 0;
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


