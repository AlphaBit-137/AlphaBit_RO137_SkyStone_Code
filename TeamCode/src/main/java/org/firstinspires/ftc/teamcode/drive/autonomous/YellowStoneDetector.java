package org.firstinspires.ftc.teamcode.drive.autonomous;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;


public class YellowStoneDetector extends DogeCVColorFilter {
    @Override
    public void process(Mat input, Mat mask) {
        Mat lab = new Mat(input.size(), 0);
        Imgproc.GaussianBlur(input,input,new Size(7,7),0);
        Imgproc.cvtColor(input, lab, Imgproc.COLOR_RGB2Lab);
        Core.inRange(lab, new Scalar(0, 120, 162), new Scalar(255, 157, 209), lab);
        Imgproc.GaussianBlur(lab, mask, new Size(3,3), 0);
        lab.release();
        input.release();
    }
}