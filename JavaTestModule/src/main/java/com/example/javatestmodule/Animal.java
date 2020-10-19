package com.example.javatestmodule;

import java.awt.Color;
import java.util.ArrayList;

public class Animal {

    public static ArrayList<Animal> animalList;
    private boolean isFast;
    public double topSpeed;
    //public String color;
    public AnimalColor color;

    //Create a getter function
    public boolean getIsFast(){
        return this.isFast;
    }

    public enum AnimalColor{
        BLUE,
        GREEN,
        PURPLE,
        BROWN,
        UNKNOWN
    }

    // Default constructor
    public Animal(){
        this.isFast = true;
        this.topSpeed = 5.5;
        //this.color = "blue";
        this.color = AnimalColor.UNKNOWN;
        if(animalList == null){
            animalList = new ArrayList<>();
        }
        animalList.add(this);
    }

    //  Overloaded constructor
    public Animal(AnimalColor assignedColor){
        this();         //Runs the default constructor first, then proceeds
        this.color = assignedColor;
    }


}
