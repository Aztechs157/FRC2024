package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.

import java.io.File;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */
public class ConfigParser {

    public SystemConfigJson systemConfigJson;

    public ConfigParser(boolean isBeta) {
        try {
            systemConfigJson = new ObjectMapper()
                    .readValue(
                            new File(Filesystem.getDeployDirectory(),
                                    isBeta ? "betasystemconfig.json" : "alphasystemconfig.json"),
                            SystemConfigJson.class);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

}
