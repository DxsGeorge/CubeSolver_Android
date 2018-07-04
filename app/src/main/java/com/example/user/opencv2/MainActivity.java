package com.example.user.opencv2;

import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.Surface;
import android.view.SurfaceView;
import android.widget.BaseAdapter;
import android.widget.TextView;
import java.util.ArrayList;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;



public class MainActivity extends AppCompatActivity implements CameraBridgeViewBase.CvCameraViewListener2 {

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
        System.loadLibrary("opencv_java");

    }

    ArrayList<Point> prevpts = new ArrayList<Point>();
    MatOfPoint prevface = new MatOfPoint();
    MatOfPoint pt = new MatOfPoint();
    MatOfPoint lastpt = new MatOfPoint();
    public static MatOfPoint2f points = new MatOfPoint2f();


    static ControlVariables controlvars = new ControlVariables();

    JavaCameraView javaCameraView;
    Mat mRGBA, mGray;
    BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case BaseLoaderCallback.SUCCESS: {
                    javaCameraView.enableView();
                    break;
                }
                default: {
                    super.onManagerConnected(status);
                    break;
                }
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        javaCameraView = (JavaCameraView) findViewById(R.id.java_camera_view);
        javaCameraView.setVisibility(SurfaceView.VISIBLE);
        javaCameraView.setCvCameraViewListener(this);
        prevpts.add(new Point (0,0));
        prevpts.add(new Point (5,0));
        prevpts.add(new Point (0,5));
        prevface.fromList(prevpts);

    }


    @Override
    protected void onPause() {
            super.onPause();
            if (javaCameraView!=null)
                javaCameraView.disableView();
        }

    @Override
    protected void onDestroy() {
            super.onDestroy();
            if (javaCameraView!=null)
                javaCameraView.disableView();
        }
    @Override
    protected void onResume() {
            super.onResume();
            if (OpenCVLoader.initDebug()) {
                mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
            }

        }

    @Override
    public void onCameraViewStarted(int width, int height) {
        mRGBA = new Mat(height,width, CvType.CV_8UC4);
    }

    @Override
    public void onCameraViewStopped() {
        mRGBA.release();
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        mRGBA = inputFrame.rgba();
        mGray = inputFrame.gray();
        long addr1 = mRGBA.getNativeObjAddr();
        long addr2 = mGray.getNativeObjAddr();
        if (addr1 == 0 || addr2 == 0)
            throw new UnsupportedOperationException("Native object address is NULL");
        else
            if (controlvars.faces_count < 6)
            {
                process(mGray.getNativeObjAddr(),
                        mRGBA.getNativeObjAddr(),
                        controlvars.tracking,
                        controlvars.detected,
                        controlvars.stat,
                        points.getNativeObjAddr(),
                        controlvars.thr,
                        controlvars.colorextract,
                        controlvars.faces_count,
                        controlvars.succ,
                        prevface.getNativeObjAddr(),
                        controlvars.undetectednum,
                        pt.getNativeObjAddr(),
                        lastpt.getNativeObjAddr());
            }
            else
            {
                //solve
            }
        return mRGBA;
    }

    public native void process(long matAddrGr,
                               long matAddrRgba,
                               int tracking,
                               int detected,
                               int stat,
                               long points_addr,
                               int thr,
                               boolean colorextract,
                               int faces_count,
                               int succ,
                               long prevface_addr,
                               int undetectedenum,
                               long pt_addr,
                               long lastpt_addr);
}
