package org.firstinspires.ftc.teamcode.Autonomous;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorDetector extends OpenCvPipeline {
    Mat mat = new Mat();
    Rect rect = new Rect(new Point(1140, 60), new Point(770, 620));
    double magentaValue;
    double yellowValue;
    double greenValue;
    private int level;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //Green
        Scalar GreenLowHSV = new Scalar(39, 50, 50);
        Scalar GreenHighHSV = new Scalar(78, 255, 255);

        //Yellow
        Scalar YellowLowHSV = new Scalar(18, 50, 50);
        Scalar YellowHighHSV = new Scalar(30, 255, 255);

        //Magenta
        Scalar MagentaLowHSV = new Scalar(130, 50, 50);
        Scalar MagentaHighHSV = new Scalar(170, 255, 255);

        Mat greenMat = mat.submat(rect);
        Mat yellowMat = mat.submat(rect);
        Mat magentaMat = mat.submat(rect);

        Core.inRange(greenMat, GreenLowHSV, GreenHighHSV, greenMat);
        Core.inRange(yellowMat, YellowLowHSV, YellowHighHSV, yellowMat);
        Core.inRange(magentaMat, MagentaLowHSV, MagentaHighHSV, magentaMat);

        Imgproc.rectangle(mat, rect, new Scalar(255, 0, 0));

        double greenValue = Core.sumElems(greenMat).val[0] / rect.area() / 255;
        double yellowValue = Core.sumElems(yellowMat).val[0] / rect.area() / 255;
        double magentaValue = Core.sumElems(magentaMat).val[0] / rect.area() / 255;

        greenMat.release();
        yellowMat.release();
        magentaMat.release();

        if(yellowValue > magentaValue && yellowValue > greenValue){
            level = 3;
        } else if(magentaValue > greenValue) {
            level = 2;
        } else {
            level = 1;
        }

        return mat;
    }

    public int getLevel(){
        return level;
    }

    public String getValues(){
        return "magenta " + magentaValue + "green " + greenValue + "yellow " + yellowValue;
    }
}