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

public class welcome extends Environment {

    public static Visitor visitor = Visitor.NULL;
    private static Boolean doorbellSounded = false;

    public static enum Visitor {
        DR_KIMBLE, POSTMAN, DELIMAN, PLUMBER, UNKNOWN, NULL
    };

    // Asynchronous subscriber that will allow code to continue. The belief is
    // directly updated in the callback function to deal with the asynchronous
    // nature.
    public void isDoorbellRingingASync() {
        AtomicReference<Boolean> status = new AtomicReference<>(false);
        bdiEnvironment.bridge.subscribe(SubscriptionRequestMsg.generate("/doorbell").setType("std_msgs/Bool")
                .setThrottleRate(1).setQueueLength(1), new RosListenDelegate() {

                    public void receive(JsonNode data, String stringRep) {
                        MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<PrimitiveMsg<String>>(
                                PrimitiveMsg.class);
                        PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
                        if ((data.get("msg").get("data").asInt() > 0) ? true : false) {
                            addPercept(Literal.parseLiteral("doorbellSounded"));
                            bdiEnvironment.logger.info("Doorbell sounded");
                        }
                    }
                });
    }

    // Example of synchronous subscriber that will pause until topic has been
    // published to, and returns the topic result
    public Boolean isDoorbellRingingSync() {
        final CountDownLatch latch = new CountDownLatch(1);
        AtomicReference<Boolean> status = new AtomicReference<>(false);

        bdiEnvironment.bridge.subscribe(SubscriptionRequestMsg.generate("/doorbell").setType("std_msgs/Bool")
                .setThrottleRate(1).setQueueLength(1), new RosListenDelegate() {

                    public void receive(JsonNode data, String stringRep) {
                        MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<PrimitiveMsg<String>>(
                                PrimitiveMsg.class);
                        PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
                        status.set(((data.get("msg").get("data").asInt() > 0) ? true : false));
                        latch.countDown();
                    }
                });

        try {
            latch.await();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return status.get();
    }

    public static void waitForBell() {
        welcome w = new welcome();
        doorbellSounded = w.isDoorbellRingingSync();
    }

    public static void scanFace() {
        visitor = Visitor.UNKNOWN;
        bdiEnvironment.logger.info("Face scan returned " + visitor.toString().toLowerCase());

    }

    public static void interrogate() {
        bdiEnvironment.logger.info("Interrogating to determine if Deliman or Plumber...");

        // Publish to HRI stack
        // Sync Subscriber to await response.
    }

    public static LinkedList<Literal> getWelcomeHomePercepts() {

        LinkedList<Literal> percepts = new LinkedList<Literal>();
        if (visitor != Visitor.NULL) {
            percepts.add(Literal.parseLiteral("personIs(" + visitor.toString().toLowerCase() + ")"));
        }
        if (doorbellSounded) {
            percepts.add(Literal.parseLiteral("doorbellSounded"));
        }

        return percepts;
    }
}
