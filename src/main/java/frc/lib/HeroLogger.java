// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import java.util.HashMap;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;

/**
 * <h3> Simple logger class that puts data on NetworkTables </h3>
 * fake advantagekit
 * <p>This fancy class looks nicer than smartdashboard. It also logs to the root of networkTables. Just use log() for anything, periodic or just once. It support structs and sendable's too.
 * This class does not have many safely nets. don't be dumb. 
 * 
 * <h4>Example: </h4>
 * <p>{@code private static HeroLogger logger = new HeroLogger("FileName");}  
 */
@SuppressWarnings({"rawtypes","unchecked"})
public class HeroLogger {
    private NetworkTable table;
    private String name;
    private final HashMap<String, StructPublisher> structs;
    private final HashMap<String, StructArrayPublisher> structArrays;

    private static HeroLogger global;
    private static HeroLogger dashboard;
    private boolean isDashboard = false;



    public HeroLogger(String name) {
        structs = new HashMap<>();
        structArrays = new HashMap<>();

        this.name = name;
        table = NetworkTableInstance.getDefault().getTable("SmartDashboard/"+this.name);
    }
    public static HeroLogger getGlobal() {
        if (global == null) global = new HeroLogger("Global");
        return global;
        
    }
    public static HeroLogger getDashboard() {
        if (dashboard == null) {
            dashboard = new HeroLogger("Dashboard");
            dashboard.isDashboard = true;
        }
        return dashboard;
    }

    public void log(String key, String value) {
        table.getEntry(key).setString(value);
    }

    public void log(String key, double value) {
        table.getEntry(key).setDouble(value);
    }
    public void log(String key, float value) {
        table.getEntry(key).setFloat(value);
    }

    public void log(String key, boolean value) {
        table.getEntry(key).setBoolean(value);
    }

    public void log(String key, Sendable value) {
        //robbed from smartdashboard
        NetworkTable dataTable = table.getSubTable(key);
        SendableBuilderImpl builder = new SendableBuilderImpl();
        builder.setTable(dataTable);
        SendableRegistry.publish(value, builder);
        builder.startListeners();
        dataTable.getEntry(".name").setString(key);
    }
    public <T> void logStruct(String key, Struct<T> struct, T object) {
        if (!structs.containsKey(key)) {
            structs.put(key,table.getStructTopic(key, struct).publish());

        } 
        structs.get(key).set(object);
    }
    public <T> void logStructArray(String key, Struct<T> struct, T[] object) {
        if (!structArrays.containsKey(key)) {
            structArrays.put(key,table.getStructArrayTopic(key, struct).publish());
        } 
        structArrays.get(key).set(object);
    }


    public boolean get(String key, boolean result) {
        return table.getEntry(key).getBoolean(result);
    }
    public double get(String key, double result) {
        return table.getEntry(key).getDouble(result);
    }
    public void get(String key, String value) {
        table.getEntry(key).getString(value);
    }



}
