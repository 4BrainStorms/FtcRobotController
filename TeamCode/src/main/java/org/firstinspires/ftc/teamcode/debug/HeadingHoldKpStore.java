package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public final class HeadingHoldKpStore {
    private static final String FILE_NAME = "heading_hold_kp.txt";

    private HeadingHoldKpStore() {}

    public static void save(double kp) {
        try {
            File f = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            ReadWriteFile.writeFile(f, String.valueOf(kp));
        } catch (Throwable ignored) {}
    }

    public static Double load() {
        try {
            File f = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            if (!f.exists()) return null;
            String s = ReadWriteFile.readFile(f).trim();
            return Double.parseDouble(s);
        } catch (Throwable ignored) {
            return null;
        }
    }
}
