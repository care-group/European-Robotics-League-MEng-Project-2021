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

	public class Room {
		public String roomName;
		LinkedList<String> doors, furniture;

		private Room(String n, LinkedList<String> d, LinkedList<String> f) {
			this.roomName = n;
			this.doors = (LinkedList<String>) d.clone();
			this.furniture = (LinkedList<String>) f.clone();
		}
	}

	public enum itemCategory {
		ROOM, DOOR, FURNITURE, OBJECT, UNKNOWN
	};

	private class Target {
		public itemCategory type;
		public String name;

		private Target() {
			type = itemCategory.UNKNOWN;
			name = null;
		}
	}

	private Logger logger = Logger.getLogger("jason_controller." + env.class.getName());
	RosBridge bridge = new RosBridge();

	// Objects to define world environment
	public static LinkedList<Room> rooms = new LinkedList<Room>();
	public static Map<String, String> objects = new LinkedHashMap<String, String>();
	public static int movedObjectFoundCounter = 0;
	public static LinkedList<Logging.Change> changes = new LinkedList<Logging.Change>();
	public static boolean changeDetected = false;
	public static Target target;

	/** Called before the MAS execution with the args informed in .mas2j */
	@Override
	public void init(String[] args) {
		super.init(args);
		bridge.connect("ws://localhost:9090", true);
		logger.info("Environment started, connection with ROS established.");
		target = new Target();
		initRooms();
		initObjects();
		printRooms();
		updatePercepts();
	}

	public void initRooms() {
		Room r1 = new Room("living_room", new LinkedList<String>(Arrays.asList("door1", "door1.2", "door1.3")),
				new LinkedList<String>(Arrays.asList("table", "sofa")));
		Room r2 = new Room("kitchen", new LinkedList<String>(Arrays.asList("door2.1", "door2.2")),
				new LinkedList<String>(Arrays.asList("fridge", "oven")));
		Room r3 = new Room("bedroom", new LinkedList<String>(Arrays.asList("door3.1")),
				new LinkedList<String>(Arrays.asList("bed", "bedsideTable", "bookshelf")));

		rooms.add(r1);
		rooms.add(r2);
		rooms.add(r3);
	}

	public void initObjects() {
		objects.put("table", "coke");
		objects.put("desk", "bag");
		objects.put("countertop", "bottle");
		objects.put("fridge", "beer");
		objects.put("bookshelf", "book");
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
					inspect(action.getTerm(0).toString());
					break;
				case "open":
					logger.info("Opening door.");
					changeDetected = false;
					rooms.getFirst().doors.removeFirst(); // remove door as we no longer care about it after opening it.
					break;
				case "find":
					if (target.type == itemCategory.FURNITURE) {
						logger.info("finding " + action.getTerm(0).toString());
						changeDetected = false;
						rooms.getFirst().furniture.removeFirst(); // remove furniture as we no longer care about it
																	// after finding it
					} else {
						logger.info("finding " + objects.get(action.getTerm(0).toString()) + " at other furniture");
						changeDetected = false;
						movedObjectFoundCounter++;
						Map.Entry<String, String> firstEntry = objects.entrySet().iterator().next();
						objects.remove(firstEntry.getKey());
					}
					break;
				case "next":
					next(action.getTerm(0).toString());
					break;
				// Temporary action to demonstrate subscriber method that waits for topic to be
				// published to.
				case "isBellRinging":
					String data = subscribeSync("/doorbell","std_msgs/Bool");
					Boolean isBellRinging = (Integer.parseInt(data) > 0) ? true : false;
					logger.info(isBellRinging.toString());
					break;
				case "saveChanges":
					Logging.saveChanges(changes, "/workspace/output/changes.txt");
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

	public void inspect(String item) {
		logger.info("Inspecting " + item);
		switch (target.type) {
			case DOOR:
				inspectDoor(item);
				break;
			case FURNITURE:
				inspectFurniture(item);
				break;
			case OBJECT:
				lookForObjectOnFurniture(item);
			default:
				break;
		}
	}

	void inspectDoor(String door) {
		// stub code that will be replaced by a topic subscriber that informs of the
		// door's position
		Random r = new Random();
		changeDetected = r.nextBoolean();

		logger.info(door + " is " + (changeDetected ? "closed" : "open"));
		if (!changeDetected) {
			rooms.getFirst().doors.removeFirst(); // remove door as we no longer care about it once
													// confirmed it's open
		} else {
			changes.push(new Logging.Change(door, itemCategory.DOOR));
		}
	}

	void inspectFurniture(String furniture) {
		// stub code that will be replaced by a topic subscriber that informs of the
		// door's position
		Random r = new Random();
		changeDetected = r.nextBoolean();

		logger.info(furniture + (changeDetected ? " has moved" : " remains untouched"));
		if (!changeDetected) {
			rooms.getFirst().furniture.removeFirst(); // remove furniture as we no longer care about it once
														// confirmed it's found
		} else {
			changes.push(new Logging.Change(furniture, itemCategory.FURNITURE));
		}
	}

	void lookForObjectOnFurniture(String furniture) {
		String object = objects.get(furniture);
		logger.info("Scanning " + furniture + " for " + object);

		// stub code that will be replaced by a topic subscriber that informs if the
		// object is detected
		Random r = new Random();
		// changeDetected = r.nextBoolean();
		changeDetected = true;
		logger.info(object + (changeDetected ? " has moved" : " has been found at " + furniture));
		if (!changeDetected) {
			objects.remove(furniture); // remove object as we no longer care about it once
										// confirmed it's found
		} else {
			changes.push(new Logging.Change(object, itemCategory.OBJECT));
		}
	}

	void printRooms() {
		for (Room room : rooms) {
			logger.info(room.roomName);
			logger.info("\tDoors");
			for (String door : room.doors) {
				logger.info("\t\t" + door);
			}
			logger.info("\tFurniture");

			for (String f : room.furniture) {
				logger.info("\t\t" + f);
			}
		}
	}

	/**
	 * Tests Jason-ROS communication by publishing to the /jason/rotate topic,
	 * causing the robot to briefly spin.
	 */
	public void test_ros_communication() {
		logger.info("Testing ROS communication");
		Publisher pub = new Publisher("/jason/rotate", "std_msgs/Int16", bridge);
		Map<String, Integer> map = new HashMap<String, Integer>();
		map.put("data", 5);
		pub.publish(map);
	}

	// Generic function to publish any type of data to a specified topic. Limited to single-member data types (eg. String)
	public <T> void publish(String topic, String type, T data) {
		logger.info("Publishing to " + topic + ", with " + type + " data.");
		Publisher pub = new Publisher(topic, type, bridge);
		Map<String, T> map = new HashMap<String, T>();
		map.put("data", data);
		pub.publish(map);
	}

	boolean checksComplete() {
		return (rooms.isEmpty());
	}

	boolean doorChecksComplete() {
		if (rooms.isEmpty()) { // prevents out of bounds error when called if rooms is empty
			return true;
		} else {
			return (rooms.getFirst().doors.isEmpty());
		}
	}

	boolean furnitureChecksComplete() {
		if (rooms.isEmpty()) { // prevents out of bounds error when called if rooms is empty
			return true;
		} else {
			return (rooms.getFirst().furniture.isEmpty());
		}
	}

	// Only to be 2 moved objects, once we have found 2 moved objects, can assume
	// this is complete, letting us end object checks early.
	boolean objectChecksComplete() {
		return (movedObjectFoundCounter >= 2);
	}

	void move_to(String location) throws Exception {
	}

	void next(String objCategory) throws Exception {
		try {
			switch (objCategory) {
				case "room":
					rooms.removeFirst();
					target.type = itemCategory.ROOM;
					target.name = rooms.getFirst().roomName;
					break;
				case "door":
					target.type = itemCategory.DOOR;
					target.name = rooms.getFirst().doors.getFirst();
					break;
				case "furniture":
					target.type = itemCategory.FURNITURE;
					target.name = rooms.getFirst().furniture.getFirst();
					break;
				case "object":
					Map.Entry<String, String> firstEntry = objects.entrySet().iterator().next();
					String location = firstEntry.getKey();
					target.type = itemCategory.OBJECT;
					target.name = location;
					break;
				default:
					logger.info("Unsure what to iterate through!");
					target.type = itemCategory.UNKNOWN;
					return;
			}
		} catch (Exception e) {
		}
	}

	// Example of synchronous subscriber that will pause until topic has been
	// published to, and returns the topic result
	Boolean isDoorbellRingingSync() {
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

	// Synchronous subscriber to a given topic, returns the output as a string
	String subscribeSync(String topic, String type){
		final CountDownLatch loginLatcch = new CountDownLatch(1);
		AtomicReference<String> status = new AtomicReference<>();

		bridge.subscribe(SubscriptionRequestMsg.generate(topic).setType(type).setThrottleRate(1)
				.setQueueLength(1), new RosListenDelegate() {

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

	// Asynchronous subscriber that will allow code to continue. The belief is
	// directly updated in the callback function to deal with the asynchronous
	// nature.
	void isDoorbellRingingASync() {
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

	/** creates the agents perception based on the MarsModel */
	void updatePercepts() {
		clearPercepts();

		isDoorbellRingingASync();

		if (checksComplete()) {
			addPercept(Literal.parseLiteral("done(rooms)"));
		}
		if (doorChecksComplete()) {
			addPercept(Literal.parseLiteral("done(doors)"));
		}
		if (furnitureChecksComplete()) {
			addPercept(Literal.parseLiteral("done(furniture)"));
		}
		if (objectChecksComplete()) {
			addPercept(Literal.parseLiteral("done(objects)"));
		}

		if (changeDetected && target.type == itemCategory.DOOR) {
			addPercept(Literal.parseLiteral("closed"));
			changeDetected = false;
		} else if (changeDetected && (target.type == itemCategory.FURNITURE || target.type == itemCategory.OBJECT)) {
			addPercept(Literal.parseLiteral("moved(" + target.name + ")"));
			changeDetected = false;
		}

		Literal target_belief;
		switch (target.type) {
			case DOOR:
				target_belief = Literal.parseLiteral("target(" + target.name + ",door)");
				break;
			case ROOM:
				target_belief = Literal.parseLiteral("target(" + target.name + ",room)");
				break;
			case FURNITURE:
				target_belief = Literal.parseLiteral("target(" + target.name + ",furniture)");
				break;
			case OBJECT:
				target_belief = Literal.parseLiteral("target(" + target.name + ",object)");
				break;
			default:
				target_belief = Literal.parseLiteral("target(" + target.name + ",unknown)");
				break;
		}

		if (target.name != null) {
			addPercept(target_belief);
		}

	}

	/** Called before the end of MAS execution */
	@Override
	public void stop() {
		super.stop();
	}

}