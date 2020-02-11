package org.firstinspires.ftc.teamcode;

import android.graphics.Rect;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.disnodeteam.dogecv.detectors.skystone.StoneDetector;
import com.disnodeteam.dogecv.filters.GrayscaleFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraBase;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Locale;

@TeleOp(name = "Skystone Detector OpMode", group="DogeCV")

public class DogeCV extends LinearOpMode {
    private OpenCvCamera webcam;
    private SkystoneDetector skyStoneDetector = new SkystoneDetector();
    private StoneDetector stoneDetector = new StoneDetector();

    private int x, y, width, height;
    private Rect rect;


    @Override
    public void runOpMode() {
        stoneDetector.useDefaults();
        stoneDetector.areaScoringMethod = com.disnodeteam.dogecv.DogeCV.AreaScoringMethod.MAX_AREA;
        stoneDetector.speed = com.disnodeteam.dogecv.DogeCV.DetectionSpeed.VERY_FAST;

        stoneDetector.filter = new LeviColorFilter(LeviColorFilter.ColorPreset.RED, 110);
        stoneDetector.stonesToFind = 1;

        skyStoneDetector.useDefaults();
        skyStoneDetector.yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 110);
        skyStoneDetector.blackFilter = new GrayscaleFilter(0, 50);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(stoneDetector);
        webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();
        }
    }
}