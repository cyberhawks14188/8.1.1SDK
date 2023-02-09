package org.firstinspires.ftc.teamcode;

import static org.opencv.core.Core.inRange;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;



@TeleOp
@Config
public class openCVTestign extends LinearOpMode
{
    public static double TSEHmin = 14, TSEHmax = 35;
    public static double TSESmin = 80, TSESmax = 255;
    public static double TSEVmin = 200, TSEVmax = 255;
    public static double poleCenterX = 160;
    public static double poleWidth = 0;
    public boolean colorTrigger = false;
    public boolean redBlueTrigger = false;
    OpenCvWebcam webcam;

    double poleCenterXPos = 160;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "IntakeCamera"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        webcam.setPipeline(new OpenCV_Pipeline());

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

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            FtcDashboard.getInstance().startCameraStream(webcam, 60);
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            //sleep(100);
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */

    public static class OpenCV_Pipeline extends OpenCvPipeline {
        openCVTestign openCVTestign = new openCVTestign();

        /** Most important section of the code: Colors **/
        static final Scalar GOLD = new Scalar(255, 215, 0);
        static final Scalar CRIMSON = new Scalar(220, 20, 60);
        static final Scalar AQUA = new Scalar(79, 195, 247);
        static final Scalar PARAKEET = new Scalar(3, 192, 74);
        static final Scalar CYAN = new Scalar(0, 139, 139);
        static final Scalar WHITE = new Scalar(255, 255, 255);

        int indexLowest; double yLowest = -10;

        double TSELocation = 0;
        public double test = 0;

        // Create a Mat object that will hold the color data


        public Rect poleBoundingBox;


        List<MatOfPoint> poleContours;

        // Make a Constructor
        public OpenCV_Pipeline() {

            poleContours = new ArrayList<MatOfPoint>();

        }

        public boolean filterContours(MatOfPoint contour) {
            return Imgproc.contourArea(contour) > 0;//not needed atm but might in the future
        }

        @Override
        public Mat processFrame(Mat input) {

            Mat YCrCb = new Mat();
            Mat HSV = new Mat();
            Mat RGBA = new Mat();

            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

            Scalar scalarLowerHSV = new Scalar(14, 80, 200);//for adjusting
            Scalar scalarUpperHSV = new Scalar(35, 255, 255);

            if(openCVTestign.colorTrigger == false){//pole detection
                scalarLowerHSV = new Scalar(14, 80, 200);//yellow
                scalarUpperHSV = new Scalar(35, 255, 255);
            }else{//for line Detection
                if(openCVTestign.redBlueTrigger == false){//blue
                    scalarLowerHSV = new Scalar(86, 75, 150);
                    scalarUpperHSV = new Scalar(124, 255, 255);
                }else{
                   // Scalar scalarLowerHSV = new Scalar(TSEHmin, TSESmin, TSEVmin);//for adjusting
                   // Scalar scalarUpperHSV = new Scalar(TSEHmax, TSESmax, TSEVmax);//for adjusting
                }
            }
            //scalarLowerHSV = new Scalar(TSEHmin, TSESmin, TSEVmin);
            //scalarUpperHSV = new Scalar(TSEHmax, TSESmax, TSEVmax);
            scalarLowerHSV = new Scalar(14, 80, 200);//yellow
            scalarUpperHSV = new Scalar(35, 255, 255);
            //Scalar scalarLowerYCrCb = new Scalar(30.0, 120.0, 75.0);//GREEN
            //Scalar scalarUpperYCrCb = new Scalar(78.0, 255.0, 255.0);//GREEN
            //Scalar scalarLowerYCrCb = new Scalar(130.0, 0.0, 50.0);//Purple
            //Scalar scalarUpperYCrCb = new Scalar(180.0, 255.0, 255.0);//Purple
            // min 0,0,0**************************************************************************************************************************************************
            // Max 180, 255,255*******************************************************************************************************************************************
            Mat maskRed = new Mat();
            //BLUE DO NOT REMOVE
            //Scalar scalarLowerYCrCb = new Scalar(80.0, 70.0, 100.0);
            //Scalar scalarUpperYCrCb = new Scalar(180.0, 255.0, 255.0);

            //inRange(HSV, lowYellow, highYellow, maskYellow);
            //inRange(HSV, lowWhite, highWhite, maskWhite);
            inRange(HSV, scalarLowerHSV, scalarUpperHSV, maskRed);

            poleContours.clear();

            Imgproc.findContours(maskRed, poleContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, poleContours, -1, AQUA); //input

            yLowest = -1;
            indexLowest = 0;
            /*Imgproc.rectangle(input, new Point(100, 225), new Point(210,375), AQUA);
            Imgproc.rectangle(input, new Point(210, 225), new Point(340, 375), PARAKEET);
            Imgproc.rectangle(input, new Point(340, 225), new Point(480, 375), GOLD);*/ //bounding boxes used for location in freight frnezy but not not needed rn

            TSELocation = 0;
            if (poleContours.size() > 0) {
                for (int i = 0; i < poleContours.size(); i++) {
                    if (filterContours(poleContours.get(i))) {
                        poleBoundingBox = Imgproc.boundingRect(poleContours.get(i));
                        Imgproc.rectangle(input, poleBoundingBox, AQUA, 10);
                        if(poleBoundingBox.height > 100) {
                            /*if ((Imgproc.contourArea(poleContours.get(i)) / (poleBoundingBox.width * poleBoundingBox.height)) > yLowest) {
                                indexLowest = i;
                                yLowest = (Imgproc.contourArea(poleContours.get(i)) / (poleBoundingBox.width * poleBoundingBox.height));
                            }*/
                           /* if (poleBoundingBox.height > yLowest) {
                                indexLowest = i;
                                yLowest = poleBoundingBox.height;
                            }*/
                            if ((poleBoundingBox.width * poleBoundingBox.height) > yLowest) {
                                indexLowest = i;
                                yLowest = (poleBoundingBox.width * poleBoundingBox.height);
                            }
                        }
                    }
                }
                poleBoundingBox = Imgproc.boundingRect(poleContours.get(indexLowest));
                Imgproc.rectangle(input, poleBoundingBox, CRIMSON, -5);


                poleCenterX = poleBoundingBox.x + (.5 * poleBoundingBox.width);
                poleWidth = poleBoundingBox.width;


            } else {
                yLowest = -1;
                poleCenterX = 0;

            }


            maskRed.release();


            YCrCb.release();
            RGBA.release();
            HSV.release();
            poleContours.clear();


            return input;
        }
    }
    public static class BlueOpenCV_Pipeline extends OpenCvPipeline {
        openCVTestign openCVTestign = new openCVTestign();

        /** Most important section of the code: Colors **/
        static final Scalar GOLD = new Scalar(255, 215, 0);
        static final Scalar CRIMSON = new Scalar(220, 20, 60);
        static final Scalar AQUA = new Scalar(79, 195, 247);
        static final Scalar PARAKEET = new Scalar(3, 192, 74);
        static final Scalar CYAN = new Scalar(0, 139, 139);
        static final Scalar WHITE = new Scalar(255, 255, 255);

        int indexLowest; double yLowest = -10;

        double TSELocation = 0;
        public double test = 0;

        // Create a Mat object that will hold the color data


        public Rect poleBoundingBox;


        List<MatOfPoint> poleContours;

        // Make a Constructor
        public BlueOpenCV_Pipeline() {

            poleContours = new ArrayList<MatOfPoint>();

        }

        public boolean filterContours(MatOfPoint contour) {
            return Imgproc.contourArea(contour) > 0;//not needed atm but might in the future
        }

        @Override
        public Mat processFrame(Mat input) {

            Mat YCrCb = new Mat();
            Mat HSV = new Mat();
            Mat RGBA = new Mat();

            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

            Scalar scalarLowerHSV = new Scalar(14, 80, 200);//for adjusting
            Scalar scalarUpperHSV = new Scalar(35, 255, 255);


            //scalarLowerHSV = new Scalar(TSEHmin, TSESmin, TSEVmin);
            //scalarUpperHSV = new Scalar(TSEHmax, TSESmax, TSEVmax);

            scalarLowerHSV = new Scalar(86, 75, 150);//BLUE
            scalarUpperHSV = new Scalar(124, 255, 255);//BLUE
            //Scalar scalarLowerYCrCb = new Scalar(30.0, 120.0, 75.0);//GREEN
            //Scalar scalarUpperYCrCb = new Scalar(78.0, 255.0, 255.0);//GREEN
            //Scalar scalarLowerYCrCb = new Scalar(130.0, 0.0, 50.0);//Purple
            //Scalar scalarUpperYCrCb = new Scalar(180.0, 255.0, 255.0);//Purple
            // min 0,0,0**************************************************************************************************************************************************
            // Max 180, 255,255*******************************************************************************************************************************************
            Mat maskRed = new Mat();
            //BLUE DO NOT REMOVE
            //Scalar scalarLowerYCrCb = new Scalar(80.0, 70.0, 100.0);
            //Scalar scalarUpperYCrCb = new Scalar(180.0, 255.0, 255.0);

            //inRange(HSV, lowYellow, highYellow, maskYellow);
            //inRange(HSV, lowWhite, highWhite, maskWhite);
            inRange(HSV, scalarLowerHSV, scalarUpperHSV, maskRed);

            poleContours.clear();

            Imgproc.findContours(maskRed, poleContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, poleContours, -1, AQUA); //input

            yLowest = -1;
            indexLowest = 0;
            /*Imgproc.rectangle(input, new Point(100, 225), new Point(210,375), AQUA);
            Imgproc.rectangle(input, new Point(210, 225), new Point(340, 375), PARAKEET);
            Imgproc.rectangle(input, new Point(340, 225), new Point(480, 375), GOLD);*/ //bounding boxes used for location in freight frnezy but not not needed rn

            TSELocation = 0;
            if (poleContours.size() > 0) {
                for (int i = 0; i < poleContours.size(); i++) {
                    if (filterContours(poleContours.get(i))) {
                        poleBoundingBox = Imgproc.boundingRect(poleContours.get(i));
                        Imgproc.rectangle(input, poleBoundingBox, AQUA, 10);
                        if(poleBoundingBox.height > 100) {
                            /*if ((Imgproc.contourArea(poleContours.get(i)) / (poleBoundingBox.width * poleBoundingBox.height)) > yLowest) {
                                indexLowest = i;
                                yLowest = (Imgproc.contourArea(poleContours.get(i)) / (poleBoundingBox.width * poleBoundingBox.height));
                            }*/
                           /* if (poleBoundingBox.height > yLowest) {
                                indexLowest = i;
                                yLowest = poleBoundingBox.height;
                            }*/
                            if ((poleBoundingBox.width * poleBoundingBox.height) > yLowest) {
                                indexLowest = i;
                                yLowest = (poleBoundingBox.width * poleBoundingBox.height);
                            }
                        }
                    }
                }
                poleBoundingBox = Imgproc.boundingRect(poleContours.get(indexLowest));
                Imgproc.rectangle(input, poleBoundingBox, CRIMSON, -5);


                poleCenterX = poleBoundingBox.x + (.5 * poleBoundingBox.width);
                poleWidth = poleBoundingBox.width;


            } else {
                yLowest = -1;
                poleCenterX = 0;

            }


            maskRed.release();


            YCrCb.release();
            RGBA.release();
            HSV.release();
            poleContours.clear();


            return input;
        }
    }
}