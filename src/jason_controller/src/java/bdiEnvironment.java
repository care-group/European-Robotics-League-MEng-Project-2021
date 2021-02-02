// Environment code
// Start rosbridge to allow Jason to ROS communication:
// roslaunch rosbridge_server rosbridge_websocket.launch

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

public class bdiEnvironment extends Environment {

	private Logger logger = Logger.getLogger("jason_controller." + env.class.getName());
	RosBridge bridge = new RosBridge();

	/** Called before the MAS execution with the args informed in .mas2j */
	@Override
	public void init(String[] args) {
		super.init(args);
		bridge.connect("ws://localhost:9090", true);
		logger.info("Environment started, connection with ROS established.");
		knowHome.init();
		updatePercepts();
	}

	@Override
	public boolean executeAction(String agName, Structure action) {
		// logger.info(agName + " doing: " + action);
		try {
			switch (action.getFunctor()) {
				case "moveTo":
					moveTo(action.getTerm(0).toString());
					break;
				case "inspect":
					knowHome.inspect(action.getTerm(0).toString());
					break;
				case "open":
					knowHome.open();
					break;
				case "find":
					knowHome.find(action.getTerm(0).toString());
					break;
				case "next":
					knowHome.next(action.getTerm(0).toString());
					break;
				// Temporary action to demonstrate subscriber method that waits for topic to be
				// published to.
				case "isBellRinging":
					String data = subscribeSync("/doorbell", "std_msgs/Bool");
					Boolean isBellRinging = (Integer.parseInt(data) > 0) ? true : false;
					logger.info(isBellRinging.toString());
					break;
				case "saveChanges":
					knowHome.saveChanges();
					break;
				case "scanFace":
					welcome.scanFace();
					break;
				default:
					logger.info("executing: " + action + ", but not implemented!");
			}
		} catch (Exception e) {
		}

		updatePercepts();
		try {
			Thread.sleep(200);
		} catch (Exception e) {
		}

		informAgsEnvironmentChanged();
		return true; // the action was executed with success
	}

	void moveTo(String location) throws Exception {
		logger.info("Moving to " + location);

	}

	// Generic function to publish any type of data to a specified topic. Limited to
	// single-member data types (eg. String)
	public <T> void publish(String topic, String type, T data) {
		logger.info("Publishing to " + topic + ", with " + type + " data.");
		Publisher pub = new Publisher(topic, type, bridge);
		Map<String, T> map = new HashMap<String, T>();
		map.put("data", data);
		pub.publish(map);
	}

	// Synchronous subscriber to a given topic, returns the output as a string
	String subscribeSync(String topic, String type) {
		final CountDownLatch latch = new CountDownLatch(1);
		AtomicReference<String> status = new AtomicReference<>();

		bridge.subscribe(SubscriptionRequestMsg.generate(topic).setType(type).setThrottleRate(1).setQueueLength(1),
				new RosListenDelegate() {

					public void receive(JsonNode data, String stringRep) {
						MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<PrimitiveMsg<String>>(
								PrimitiveMsg.class);
						PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
						status.set(data.get("msg").get("data").asText());
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

	/** creates the agents perception based on the MarsModel */
	void updatePercepts() {
		clearPercepts();
		LinkedList<Literal> knowMyHomePercepts = knowHome.getKnowMyHomePercepts();
		addPercepts(knowMyHomePercepts);
	}

	private void addPercepts(LinkedList<Literal> literals) {
		for (Literal l : literals) {
			addPercept(l);
		}
	}

	/** Called before the end of MAS execution */
	@Override
	public void stop() {
		super.stop();
	}

}