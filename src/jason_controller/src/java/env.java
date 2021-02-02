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

public class env extends Environment {

	private Logger logger = Logger.getLogger("jason_controller." + env.class.getName());
	RosBridge bridge = new RosBridge();

	/** Called before the MAS execution with the args informed in .mas2j */
	@Override
	public void init(String[] args) {
		super.init(args);
		bridge.connect("ws://localhost:9090", true);
		logger.info("Environment started, connection with ROS established.");
		know_home.init();
		updatePercepts();
	}

	@Override
	public boolean executeAction(String agName, Structure action) {
		// logger.info(agName + " doing: " + action);
		try {
			switch (action.getFunctor()) {
				case "test_ros_communication":
					publish("/jason/genericBool", "std_msgs/Bool", true);
					break;
				case "move_to":
					logger.info("Moving to " + action.getTerm(0));
					move_to(action.getTerm(0).toString());
					break;
				case "inspect":
					know_home.inspect(action.getTerm(0).toString());
					break;
				case "open":
					know_home.open();
					break;
				case "find":
					know_home.find(action.getTerm(0).toString());
					break;
				case "next":
					know_home.next(action.getTerm(0).toString());
					break;
				// Temporary action to demonstrate subscriber method that waits for topic to be
				// published to.
				case "isBellRinging":
					String data = subscribeSync("/doorbell", "std_msgs/Bool");
					Boolean isBellRinging = (Integer.parseInt(data) > 0) ? true : false;
					logger.info(isBellRinging.toString());
					break;
				case "saveChanges":
					know_home.saveChanges();
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

	void move_to(String location) throws Exception {
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
		final CountDownLatch loginLatcch = new CountDownLatch(1);
		AtomicReference<String> status = new AtomicReference<>();

		bridge.subscribe(SubscriptionRequestMsg.generate(topic).setType(type).setThrottleRate(1).setQueueLength(1),
				new RosListenDelegate() {

					public void receive(JsonNode data, String stringRep) {
						MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<PrimitiveMsg<String>>(
								PrimitiveMsg.class);
						PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
						status.set(data.get("msg").get("data").asText());
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

	private void updateGettingToKnowMyHomePercepts() {
		if (know_home.checksComplete()) {
			addPercept(Literal.parseLiteral("done(rooms)"));
		}
		if (know_home.doorChecksComplete()) {
			addPercept(Literal.parseLiteral("done(doors)"));
		}
		if (know_home.furnitureChecksComplete()) {
			addPercept(Literal.parseLiteral("done(furniture)"));
		}
		if (know_home.objectChecksComplete()) {
			addPercept(Literal.parseLiteral("done(objects)"));
		}

		if (know_home.getChangeDetected() && know_home.getTarget().targetIs(know_home.itemCategory.DOOR)) {
			addPercept(Literal.parseLiteral("closed"));
			know_home.setChangeDetected(false);

		} else if (know_home.getChangeDetected() && (know_home.getTarget().targetIs(know_home.itemCategory.FURNITURE)
				|| know_home.getTarget().targetIs(know_home.itemCategory.OBJECT))) {
			addPercept(Literal.parseLiteral("moved(" + know_home.getTarget().name + ")"));
			know_home.setChangeDetected(false);
		}

		Literal target_belief;
		switch (know_home.getTarget().type) {
			case DOOR:
				target_belief = Literal.parseLiteral("target(" + know_home.getTarget().name + ",door)");
				break;
			case ROOM:
				target_belief = Literal.parseLiteral("target(" + know_home.getTarget().name + ",room)");
				break;
			case FURNITURE:
				target_belief = Literal.parseLiteral("target(" + know_home.getTarget().name + ",furniture)");
				break;
			case OBJECT:
				target_belief = Literal.parseLiteral("target(" + know_home.getTarget().name + ",object)");
				break;
			default:
				target_belief = Literal.parseLiteral("target(" + know_home.getTarget().name + ",unknown)");
				break;
		}

		if (!know_home.getTarget().isEmpty()) {
			addPercept(target_belief);
		}

	}

	/** creates the agents perception based on the MarsModel */
	void updatePercepts() {
		clearPercepts();
		updateGettingToKnowMyHomePercepts();

	}

	/** Called before the end of MAS execution */
	@Override
	public void stop() {
		super.stop();
	}

}