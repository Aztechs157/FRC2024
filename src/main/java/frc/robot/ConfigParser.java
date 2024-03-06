package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.

import java.io.File;

import com.fasterxml.jackson.databind.ObjectMapper;

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */
public class ConfigParser {

    public static SystemConfigJson systemConfigJson;

    public ConfigParser(File directory) {
        try {
            systemConfigJson = new ObjectMapper()
                    .readValue(new File(directory, "systemConfig.json"), SystemConfigJson.class);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

}
