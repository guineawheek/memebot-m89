package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class VuMarkRecognition
{
    VuforiaLocalizer vuforia;
    VuforiaTrackable relicTemplate;
    VuforiaTrackables relicTrackables;

    public VuMarkRecognition(HardwareMap hardwareMap, Telemetry telemetry){
        init(hardwareMap, telemetry);
    }
    public void init(HardwareMap hardwareMap, Telemetry telemetry)
    {
        telemetry.addData("starting vuforia","");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AWFEfJ//////AAAAGRSAEvItIE6VmtdgqLncdq5pwXP81G5X4IT2ssIXnwbRECRsNIbxeFqkExyfeZM/uFLLMbwShZBYACYOOgl/aIz8PqlIV8FSGppU1XkPuC9WjGLiclIsgO+AXZ4OEKJyc27eiNvXTNI8MTBxyR3vk/cB9XDqtC7ksqhB8TgFD9QuKS3Xo9gOH8edNZ+pD6T0Xjfbh3Vl8REKuTcCQvIDG3ImRJVi3b6fsXxmciBv+pw91FqMjZeZbHFEKrDLlUHisvgQ1NkaoiOlyWw5XCXFmv0gtO0t+whwTOMz3dkOfUPafS+e2oDdd5SQJgk6R7SgzTmc/8Ld/TqiZtXSsIR39qUFdklNGhmWqd+mA5ZNCLUq";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; //Use selfie camera
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();

        telemetry.addData("finished vuforia","");
        telemetry.update();
    }

    //0 - Unknown
    //1 - Left
    //2 - Center
    //3 - Right
    public int getVuMark()
    {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        switch(vuMark)
        {
            case UNKNOWN:
                return 0;
            case LEFT:
                return 1;
            case CENTER:
                return 2;
            case RIGHT:
                return 3;
            default:
                return 2;
        }
    }
}