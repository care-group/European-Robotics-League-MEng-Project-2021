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

	public static Logger logger = Logger.getLogger("jason_controller." + env.class.getName());
	public static RosBridge bridge = new RosBridge();

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
				/* === COMMON === */
				case "moveTo":
					moveTo(action.getTerm(0).toString());
					break;
				case "escort":
					escort(action.getTerm(0).toString());
					break;
				case "open":
					knowHome.openDoor();
					break;
				case "closeDoor":
					closeDoor();
					break;
				/* === GETTING TO KNOW MY HOME === */
				case "inspect":
					knowHome.inspect(action.getTerm(0).toString());
					break;
				case "find":
					knowHome.find(action.getTerm(0).toString());
					break;
				case "next":
					knowHome.next(action.getTerm(0).toString());
					break;
				case "saveChanges":
					knowHome.saveChanges();
					break;
				/* === WELCOMING VISITORS === */
				case "waitForBell":
					welcome.waitForBell();
					break;
				case "scanFace":
					welcome.scanFace();
					break;
				case "interrogate":
					welcome.interrogate();
					break;
				case "waitUntilVisitorDone":
					welcome.waitUntilVisitorDone();
					break;
				case "acceptMail":
					welcome.acceptMail();
					break;
				case "deliverMail":
					welcome.deliverMail();
					break;
				case "complain":
					welcome.complain();
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

	void escort(String location) {
		logger.info("Escorting to " + location);
	}

	void closeDoor() {
		logger.info("Closing door.");
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
	static String subscribeSync(String topic, String type) {
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

	// Asynchronous subscriber to a given boolean topic, adding a given percept if
	// true.
	public void subscribeASync(String topic, String literal) {
		bdiEnvironment.bridge.subscribe(
				SubscriptionRequestMsg.generate(topic).setType("std_msgs/Bool").setThrottleRate(1).setQueueLength(1),
				new RosListenDelegate() {

					public void receive(JsonNode data, String stringRep) {
						MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<PrimitiveMsg<String>>(
								PrimitiveMsg.class);
						PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
						if ((data.get("msg").get("data").asInt() > 0)) {
							addPercept(Literal.parseLiteral(literal));
						}
					}
				});
	}

	/** creates the agents perception based on the MarsModel */
	void updatePercepts() {

		clearPercepts();

		callASyncSubscribers();

		LinkedList<Literal> knowMyHomePercepts = knowHome.getKnowMyHomePercepts();
		LinkedList<Literal> welcomePercepts = welcome.getWelcomeHomePercepts();
		addPercepts(knowMyHomePercepts);
		addPercepts(welcomePercepts);
	}

	private void addPercepts(LinkedList<Literal> literals) {
		if (!literals.isEmpty()) {
			for (Literal l : literals) {
				addPercept(l);
			}
		}
	}

	private void callASyncSubscribers() {
		subscribeASync("/jason/welcome/visitorOutOfBounds", "visitorOutOfBounds");
	}

	/** Called before the end of MAS execution */
	@Override
	public void stop() {
		super.stop();
	}

}