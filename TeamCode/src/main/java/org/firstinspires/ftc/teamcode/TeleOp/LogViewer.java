package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Helper.GamePad;

import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

@Config
@TeleOp(name = "LogViewer", group = "TeleOp")
public class LogViewer extends LinearOpMode{

    public static String url = "";

    public static int fileGet = 1;

    public static String type = "customFile";

    private final GamePad gpInput = new GamePad(gamepad1, false);

    public void runOpMode () {
        waitForStart();
        if (isStopRequested()) return;

        telemetry.clear();

        telemetry.addLine("A - many files, B - custom file, X - Browse Files");
        telemetry.update();

        while (opModeIsActive()) {
            GamePad.GameplayInputType inputType = gpInput.WaitForGamepadInput(30);
            switch (inputType) {
                case BUTTON_A:
                    LogViewer.type = "manyFiles";
                    this.runLogView();
                    break;
                case BUTTON_B:
                    LogViewer.type = "customFile";
                    this.runLogView();
                    break;
                case BUTTON_X:
                    LogViewer.type = "browse";
                    this.runLogView();
                    break;
            }
        }
    }

    private void runLogView() {
        String directoryPath = "RoadRunner/logs";
        File directory = new File(AppUtil.ROOT_FOLDER, directoryPath);

        try {
            tryUploadingBrowseFiles(getLogFiles(directoryPath));
        } catch (Exception e) {
            telemetry.addLine(e.toString());
            telemetry.update();
            e.printStackTrace();
            return;
        }

        if (type.equals("browse")) {
            telemetry.addLine("File Browser Cache Updated");
            telemetry.update();
            return;
        }

        if (type.equals("customFile")) {
            try {
                String filename = getFileName();
                if ((filename == null) || (filename.isEmpty())) {
                    telemetry.addLine("filename is null");
                }

                File file = getSpecificFile(directory, filename);

                if (file != null) {
                    boolean success = uploadFile(file, LogViewer.url+"upload");
                    if (success) {
                        telemetry.addLine("\nSuccessfully uploaded file: "+file.getName());
                    } else  {
                        telemetry.addLine("\nFile Upload Failed: "+file.getName());
                    }
                }

            } catch (Exception e) {
                telemetry.addLine(e.toString());
                telemetry.update();
                e.printStackTrace();
                return;
            }

            telemetry.update();
            return;
        }

        if (directory.exists() && directory.isDirectory()) {
            if (fileGet == 1) {
                File latestLogFile = getLatestLogFile(directory);
                if (latestLogFile != null) {
                    boolean success = uploadFile(latestLogFile, url+"upload");
                    if (success) {
                        telemetry.addLine("\nSuccessfully uploaded file: "+latestLogFile.getName());
                    } else  {
                        telemetry.addLine("\nFile Upload Failed: "+latestLogFile.getName());
                    }
                }
            } else {
                File[] files = getFiles(directory);
                for (File latestLogFile : files) {
                    if (latestLogFile != null) {
                        boolean success = uploadFile(latestLogFile, url+"upload");
                        if (success) {
                            telemetry.addLine("\nSuccessfully uploaded file: "+latestLogFile.getName());
                        } else  {
                            telemetry.addLine("\nFile Upload Failed: "+latestLogFile.getName());
                        }
                    }
                }
            }
        }

        telemetry.update();
        return;
    }

    public static String getFileName() throws Exception {
        @Deprecated
        URL url = new URL(LogViewer.url+"getfilename");
        HttpURLConnection connection = (HttpURLConnection) url.openConnection();
        connection.setRequestMethod("GET");
        connection.setRequestProperty("Accept", "application/json");

        int responseCode = connection.getResponseCode();
        if (responseCode == HttpURLConnection.HTTP_OK) {
            try (BufferedReader in = new BufferedReader(new InputStreamReader(connection.getInputStream()))) {
                StringBuilder response = new StringBuilder();
                String inputLine;

                while ((inputLine = in.readLine()) != null) {
                    response.append(inputLine);
                }

                return response.toString();
            }
        } else {
            return null;
        }
    }

    public static String toJsonWithFilesKey(ArrayList<String> list) {
        StringBuilder jsonBuilder = new StringBuilder();
        jsonBuilder.append("{\"files\":[");

        for (int i = 0; i < list.size(); i++) {
            jsonBuilder.append("\"").append(list.get(i)).append("\"");
            if (i < list.size() - 1) {
                jsonBuilder.append(",");
            }
        }

        jsonBuilder.append("]}");
        return jsonBuilder.toString();
    }

    public void tryUploadingBrowseFiles(ArrayList<String> logFiles) throws Exception {
        String jsonPayload = toJsonWithFilesKey(logFiles);

        @Deprecated
        URL url = new URL(LogViewer.url + "updatefiles");

        HttpURLConnection connection = (HttpURLConnection) url.openConnection();
        connection.setRequestMethod("POST");
        connection.setRequestProperty("Content-Type", "application/json; utf-8");
        connection.setRequestProperty("Accept", "application/json");
        connection.setDoOutput(true);

        try (OutputStream os = connection.getOutputStream()) {
            byte[] input = jsonPayload.getBytes("utf-8");
            os.write(input, 0, input.length);
        }

        int responseCode = connection.getResponseCode();
        if (responseCode == HttpURLConnection.HTTP_OK) {
        } else {
        }
    }

    public static ArrayList<String> getLogFiles(String directoryPath) {
        File directory = new File(directoryPath);
        ArrayList<String> logFiles = new ArrayList<>();

        if (directory.exists() && directory.isDirectory()) {
            File[] files = directory.listFiles((dir, name) -> name.endsWith(".log"));

            if (files != null) {
                Arrays.sort(files, Comparator.comparingLong(File::lastModified));

                for (File file : files) {
                    logFiles.add(file.getName());
                }
            }
        }

        return logFiles;
    }

    private boolean uploadFile(File file, String param_url) {
        HttpURLConnection connection = null;
        DataOutputStream outputStream = null;

        String boundary = "*****";
        String lineEnd = "\r\n";
        String twoHyphens = "--";

        try {
            FileInputStream fileInputStream = new FileInputStream(file);

            @Deprecated
            URL url = new URL(param_url);

            connection = (HttpURLConnection) url.openConnection();
            connection.setDoInput(true);
            connection.setDoOutput(true);
            connection.setRequestMethod("POST");
            connection.setRequestProperty("Connection", "Keep-Alive");
            connection.setRequestProperty("Content-Type", "multipart/form-data;boundary=" + boundary);
            connection.setRequestProperty("file", file.getName());

            outputStream = new DataOutputStream(connection.getOutputStream());



            outputStream.writeBytes(twoHyphens + boundary + lineEnd);
            outputStream.writeBytes("Content-Disposition: form-data; name=\"file\";filename=\"" + file.getName() + "\"" + lineEnd);
            outputStream.writeBytes(lineEnd);

            int bytesRead;
            byte[] buffer = new byte[1024];
            while ((bytesRead = fileInputStream.read(buffer)) != -1) {
                outputStream.write(buffer, 0, bytesRead);
            }

            outputStream.writeBytes(lineEnd);
            outputStream.writeBytes(twoHyphens + boundary + twoHyphens + lineEnd);

            fileInputStream.close();
            outputStream.flush();
            outputStream.close();

            int responseCode = connection.getResponseCode();
            return (responseCode == HttpURLConnection.HTTP_OK);

        } catch (IOException e) {
            e.printStackTrace();
            return false;
        } finally {
            if (connection != null) {
                connection.disconnect();
            }
        }
    }

    private File[] getFiles(File directory) {
        File[] files = directory.listFiles((dir, name) -> name.toLowerCase().endsWith(".log"));

        if (files != null) {
            Arrays.sort(files, Comparator.comparingLong(File::lastModified).reversed());

            return Arrays.copyOf(files, Math.min(files.length, fileGet));
        }

        return new File[0];
    }


    private File getSpecificFile(File directory, String fileName) {
        File[] files = directory.listFiles((dir, name) -> name.toLowerCase().endsWith(".log"));

        if (files != null) {
            for (File file : files) {
                String substring = fileName.trim().substring(1, fileName.length() - 1);
                if ((file.getName()).equals(substring)) {
                    return file;
                }
            }
        }

        return null;
    }

    private File getLatestLogFile(File directory) {
        File[] files = directory.listFiles((dir, name) -> name.toLowerCase().endsWith(".log"));
        File latestFile = null;
        long lastModified = Long.MIN_VALUE;

        if (files != null) {
            for (File file : files) {
                if (file.lastModified() > lastModified) {
                    lastModified = file.lastModified();
                    latestFile = file;
                }
            }
        }
        return latestFile;
    }
}