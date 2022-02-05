package org.firstinspires.ftc.teamcode.sandboxes.Dennis;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ml.HubModel;
import org.tensorflow.lite.DataType;
import org.tensorflow.lite.support.tensorbuffer.TensorBuffer;

import java.io.IOException;
import java.nio.ByteBuffer;

@Disabled
@TeleOp(name = "B - Tensorflow Testing (test)!")
public class TensorflowTesting extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();

        while(!isStopRequested()) {

            ByteBuffer byteBuffer = ByteBuffer.allocate(256);
            try {
                HubModel model = HubModel.newInstance(hardwareMap.appContext);

                // Inputs
                TensorBuffer inputFeature0 = TensorBuffer.createFixedSize(new int[]{1, 224, 224, 3}, DataType.FLOAT32);
                inputFeature0.loadBuffer(byteBuffer);

                // run it through
                HubModel.Outputs outputs = model.process(inputFeature0);
                TensorBuffer outputFeature0 = outputs.getOutputFeature0AsTensorBuffer();

                // Releases model resources if no longer used.
                model.close();

            } catch(IOException e) {

            }
        }
    }


}
