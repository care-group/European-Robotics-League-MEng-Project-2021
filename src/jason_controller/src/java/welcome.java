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

    private static Logger logger = Logger.getLogger("jason_controller." + env.class.getName());
    private RosBridge bridge = new RosBridge();

    // Asynchronous subscriber that will allow code to continue. The belief is
    // directly updated in the callback function to deal with the asynchronous
    // nature.
    public void isDoorbellRingingASync() {
        AtomicReference<Boolean> status = new AtomicReference<>(false);
        bridge.subscribe(SubscriptionRequestMsg.generate("/doorbell").setType("std_msgs/Bool").setThrottleRate(1)
                .setQueueLength(1), new RosListenDelegate() {

                    public void receive(JsonNode data, String stringRep) {
                        MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<PrimitiveMsg<String>>(
                                PrimitiveMsg.class);
                        PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);

                        if ((data.get("msg").get("data").asInt() > 0) ? true : false) {
                            addPercept(Literal.parseLiteral("doorbellSounded"));
                        }
                    }
                });
    }

    // Example of synchronous subscriber that will pause until topic has been
    // published to, and returns the topic result
    public Boolean isDoorbellRingingSync() {
        final CountDownLatch loginLatcch = new CountDownLatch(1);
        AtomicReference<Boolean> status = new AtomicReference<>(false);

        bridge.subscribe(SubscriptionRequestMsg.generate("/doorbell").setType("std_msgs/Bool").setThrottleRate(1)
                .setQueueLength(1), new RosListenDelegate() {

                    public void receive(JsonNode data, String stringRep) {
                        MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<PrimitiveMsg<String>>(
                                PrimitiveMsg.class);
                        PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
                        status.set(((data.get("msg").get("data").asInt() > 0) ? true : false));
                        loginLatcch.countDown();
                    }
                });

        try {
            loginLatcch.await();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return status.get();
    }
}
