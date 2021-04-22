package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class ImageDetector extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    public String position = "LEFT";

    public ImageDetector() {
    }

    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if(workingMatrix.empty()) {
            return input;
        }
        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        Mat matLeft = workingMatrix.submat(150, 180, 10, 50);
        Mat matCenter = workingMatrix.submat(150, 180, 80, 120);
        Mat matRight = workingMatrix.submat(150, 180, 150, 190);

        Imgproc.rectangle(workingMatrix, new Rect(10, 150, 40, 30), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(80, 150, 40, 30), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(150, 150, 40, 30), new Scalar(0, 255, 0));


        double leftTotal = Core.sumElems(matLeft).val[2];
        double centerTotal = Core.sumElems(matCenter).val[2];
        double rightTotal = Core.sumElems(matRight).val[2];

        if(leftTotal > centerTotal) {
            if(leftTotal > rightTotal) {
                ///left
                position = "LEFT";
            }
            else {
                ///right
                position = "RIGHT";
            }

        }
        else {
            if(centerTotal > rightTotal) {
                //center
                position = "CENTER";
            }
            else {
                ///right
                position = "RIGHT";
            }
        }

        return workingMatrix;

    }
}
