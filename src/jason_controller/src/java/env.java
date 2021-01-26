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

	public enum targetEnum {
		ROOM, DOOR, FURNITURE, UNKNOWN
	};

	private class Target {
		public targetEnum type;
		public String name;

		private Target() {
			type = targetEnum.UNKNOWN;
			name = null;
		}
	}

	private Logger logger = Logger.getLogger("jason_controller." + env.class.getName());
	RosBridge bridge = new RosBridge();

	// Objects to define world environment
	public static LinkedList<Room> rooms = new LinkedList<Room>();
	public static LinkedList<String> changes = new LinkedList<String>();
	public static boolean changeDetected = false;

	public static Target target;

	/** Called before the MAS execution with the args informed in .mas2j */
	@Override
	public void init(String[] args) {
		super.init(args);
		// bridge.connect("ws://localhost:9090", true);
		logger.info("Environment started, connection with ROS established.");
		target = new Target();
		initRooms();
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

	@Override
	public boolean executeAction(String agName, Structure action) {
		// logger.info(agName + " doing: " + action);
		try {
			switch (action.getFunctor()) {
				case "test_ros_communication":
					test_ros_communication();
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
					changeDetected = false;
					rooms.getFirst().furniture.removeFirst(); // remove furniture as we no longer care about it after
																// finding it
					break;
				case "next":
					next(action.getTerm(0).toString());
					break;
				case "saveChanges":
					fileIO.saveChanges(changes, "/workspace/output/changes.txt");
					break;
				default:
					logger.info("executing: " + action + ", but not implemented!");
			}
		} catch (Exception e) {
			e.printStackTrace();
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
			changes.push(door);
		}
		target.name = null; // reset target to avoid target being repeated
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
			changes.push(furniture);
		}
		target.name = null;
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

	void move_to(String location) throws Exception {
	}

	void next(String objCategory) throws Exception {
		try {
			switch (objCategory) {
				case "room":
					rooms.removeFirst();
					target.type = targetEnum.ROOM;
					target.name = rooms.getFirst().roomName;
					break;
				case "door":
					target.type = targetEnum.DOOR;
					target.name = rooms.getFirst().doors.getFirst();
					break;
				case "furniture":
					target.type = targetEnum.FURNITURE;
					target.name = rooms.getFirst().furniture.getFirst();
					break;
				default:
					logger.info("Unsure what to iterate through!");
					target.type = targetEnum.UNKNOWN;
					return;
			}
		} catch (Exception e) {
		}
	}

	/** creates the agents perception based on the MarsModel */
	void updatePercepts() {
		clearPercepts();
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
			default:
				target_belief = Literal.parseLiteral("target(" + target.name + ",unknown)");
				break;
		}

		if (checksComplete()) {
			addPercept(Literal.parseLiteral("done(rooms)"));
		}
		if (doorChecksComplete()) {
			addPercept(Literal.parseLiteral("done(doors)"));
		}
		if (furnitureChecksComplete()) {
			addPercept(Literal.parseLiteral("done(furniture)"));
		}

		if (target.name != null) {
			addPercept(target_belief);
		}

		if (changeDetected && target.type == targetEnum.DOOR) {
			addPercept(Literal.parseLiteral("closed"));
		} else if (changeDetected && target.type == targetEnum.FURNITURE) {
			addPercept(Literal.parseLiteral("moved(" + target.name + ")"));
		}
	}

	/** Called before the end of MAS execution */
	@Override
	public void stop() {
		super.stop();
	}

}