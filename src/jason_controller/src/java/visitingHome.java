import jason.asSyntax.*;
import jason.environment.*;

import java.util.logging.*;
import ros.Publisher;
import ros.RosBridge;
import ros.RosListenDelegate;
import ros.SubscriptionRequestMsg;
import ros.msgs.std_msgs.PrimitiveMsg;
import ros.tools.MessageUnpacker;
import com.fasterxml.jackson.databind.JsonNode;

import java.util.*;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class visitingHome extends Environment {

    private static Boolean entranceOpened = false;

    private static enum Obstacle {
        SMALL, WHEELED,HUMAN
    };
    private static Obstacle obstacle;

    public static void waitForEntranceOpened() {
        bdiEnvironment.logger.info("Waiting for front door to be opened...");
        String rawStatus = bdiEnvironment.subscribeSync("/jason/visitingHome/entranceOpened", "std_msgs/Bool");
        entranceOpened = rawStatus == "true" ? true : false;
    }



    public static void identifyObstacle() {
        bdiEnvironment.logger.info("Identifying obstacle...");
        //get position
        //get type
        obstacle=Obstacle.SMALL;
    }

    public static void moveObstacle() {
        bdiEnvironment.logger.info("Dealing with obstacle.");
        switch(obstacle){
            case SMALL:
                //pick up and place
            break;
            case WHEELED:
                //push out the way
            break;
            case HUMAN:
                //please move
            break;
        }
    }


    public static LinkedList<Literal> getVisitingHomePercepts() {

        LinkedList<Literal> percepts = new LinkedList<Literal>();

        if (entranceOpened) {
            percepts.add(Literal.parseLiteral("entranceOpened"));
        }
   

        return percepts;
    }
}
