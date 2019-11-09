package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.nio.ByteBuffer;

public class SkyStoneFinder {
    public int getStonePos() throws Exception {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = "AYWINAP/////AAABmSVF7mbdHUEqp97T/zt9WtdjFYMPVsj/w5FhyKAWdPbjm6LPrvu7rETQCmDzUbhspfp+PK+7S0YzJkOT3mPlxDY3zsquMCi3Yhjop9uuAJbSiWHlpEjUlB5sAF6pLFSeZYALOVZHHJkWlb8dxIukVmXKOa4MIMTzjSlJDA6shu4KaViz+P6t5cSmDS0CY/RQBD4b2Ciuf5sYQb05hyAYyZ4fkvyNJ4Q0oqH0AJQh7qTwWfSKToDqwJn8Pryas+dI8EZ+Kd/+t/0mZdLD2UhknAT2+aisB6p62qjGIyHXC7dXKlx2G3a+ggcurNpv+NDE/5rxOwif19jhhG5u8oMbhnD4b8k4P3OmcXJE0G3yq0J1";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        VuforiaLocalizer locale = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        locale.setFrameQueueCapacity(1);

        VuforiaLocalizer.CloseableFrame frame;

        frame = locale.getFrameQueue().take();
        Image rgb = null;

        long numImages = frame.getNumImages();

        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }

        ByteBuffer bb = rgb.getPixels();
        byte[] bytes = new byte[bb.capacity()];
        bb.get(bytes);
        Bitmap source = BitmapFactory.decodeByteArray(bytes, 0, bytes.length, null);

        int y = rgb.getHeight() / 2;

        int blocks = 0;

        int blockLength = source.getWidth() / 4;

        int temp = 0;
        int tempLight = 0;
        for (int i = 0; i < source.getWidth(); i++) {
            int pixel = source.getPixel(i, y);
            int r = Color.red(pixel);
            int g = Color.green(pixel);
            int b = Color.blue(pixel);

            int gray = (r + b + g) / 3;

            if (gray <= 80) {
                temp++;
                if (tempLight > blockLength) {
                    i = source.getWidth();
                }
                tempLight = 0;
            } else {
                tempLight++;
                if (temp > blockLength) {
                    blocks++;
                }
                temp = 0;
            }
        }

        return blocks;
    }
}
