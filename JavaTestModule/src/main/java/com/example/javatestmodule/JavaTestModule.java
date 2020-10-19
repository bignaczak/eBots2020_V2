package com.example.javatestmodule;

import java.util.ArrayList;
import java.util.EnumSet;

public class JavaTestModule {

    public enum Tester {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    public static void main(String[] args) {
        String myName = "Shaan";
        System.out.println("Hello " + myName + "!");

        Animal myAnimal = new Animal();

        System.out.print("Is the animal fast? ");
        System.out.println(myAnimal.getIsFast());

        Animal myFrog = new Animal(Animal.AnimalColor.GREEN);
        System.out.println("This animal is a frog he is " + myFrog.color);

        Animal animal1 = new Animal(Animal.AnimalColor.PURPLE);
        Animal animal2 = new Animal(Animal.AnimalColor.BLUE);


        boolean isMatch;
        int i = 2;
        int j = 2;
        Animal.AnimalColor color1 = animal1.color;
        Animal.AnimalColor color2 = animal2.color;
        isMatch = (color1 == color2);
        System.out.print("The values match? ");
        System.out.println(isMatch);

        double forward = 0;
        double lateral = -1;

        double angle = Math.atan2(lateral, forward);
        System.out.println(Math.toDegrees(angle));

        for (Tester t : Tester.values()) {
            System.out.println(t);
        }


    }
}