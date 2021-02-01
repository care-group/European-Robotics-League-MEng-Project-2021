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

public class getting_to_know_my_home extends Environment {

	public static class Room {
		public String roomName;
		LinkedList<String> doors, furniture;

		private Room(String n, LinkedList<String> d, LinkedList<String> f) {
			this.roomName = n;
			this.doors = (LinkedList<String>) d.clone();
			this.furniture = (LinkedList<String>) f.clone();
		}
	}

	public static enum itemCategory {
		ROOM, DOOR, FURNITURE, OBJECT, UNKNOWN
	};

	private static class Target {
		public itemCategory type;
		public String name;

		private Target() {
			type = itemCategory.UNKNOWN;
			name = null;
		}
	}

	private static Logger logger = Logger.getLogger("jason_controller." + env.class.getName());

	// Objects to define world environment
	private static LinkedList<Room> rooms = new LinkedList<Room>();
	private static Map<String, String> objects = new LinkedHashMap<String, String>();
	private static int movedObjectFoundCounter = 0;
	private static LinkedList<Logging.Change> changes = new LinkedList<Logging.Change>();
	private static boolean changeDetected = false;
	private static Target target;

	public static void init() {
		target = new Target();
		initRooms();
		initObjects();
		printRooms();
	}

	private static void initRooms() {
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

	private static void initObjects() {
		objects.put("table", "coke");
		objects.put("desk", "bag");
		objects.put("countertop", "bottle");
		objects.put("fridge", "beer");
		objects.put("bookshelf", "book");
	}

	public static void inspect(String item) {
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

	private static void inspectDoor(String door) {
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

	private static void inspectFurniture(String furniture) {
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

	private static void lookForObjectOnFurniture(String furniture) {
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

	private static void printRooms() {
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

	private static boolean checksComplete() {
		return (rooms.isEmpty());
	}

	private static boolean doorChecksComplete() {
		if (rooms.isEmpty()) { // prevents out of bounds error when called if rooms is empty
			return true;
		} else {
			return (rooms.getFirst().doors.isEmpty());
		}
	}

	private static boolean furnitureChecksComplete() {
		if (rooms.isEmpty()) { // prevents out of bounds error when called if rooms is empty
			return true;
		} else {
			return (rooms.getFirst().furniture.isEmpty());
		}
	}

	// Only to be 2 moved objects, once we have found 2 moved objects, can assume
	// this is complete, letting us end object checks early.
	private static boolean objectChecksComplete() {
		return (movedObjectFoundCounter >= 2);
	}

	public static void next(String objCategory) throws Exception {
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

	/** creates the agents perception based on the MarsModel */
	// public void updatePercepts() {

	// if (checksComplete()) {
	// addPercept(Literal.parseLiteral("done(rooms)"));
	// }
	// if (doorChecksComplete()) {
	// addPercept(Literal.parseLiteral("done(doors)"));
	// }
	// if (furnitureChecksComplete()) {
	// addPercept(Literal.parseLiteral("done(furniture)"));
	// }
	// if (objectChecksComplete()) {
	// addPercept(Literal.parseLiteral("done(objects)"));
	// }

	// if (changeDetected && target.type == itemCategory.DOOR) {
	// addPercept(Literal.parseLiteral("closed"));
	// changeDetected = false;
	// } else if (changeDetected && (target.type == itemCategory.FURNITURE ||
	// target.type == itemCategory.OBJECT)) {
	// addPercept(Literal.parseLiteral("moved(" + target.name + ")"));
	// changeDetected = false;
	// }

	// Literal target_belief;
	// switch (target.type) {
	// case DOOR:
	// target_belief = Literal.parseLiteral("target(" + target.name + ",door)");
	// break;
	// case ROOM:
	// target_belief = Literal.parseLiteral("target(" + target.name + ",room)");
	// break;
	// case FURNITURE:
	// target_belief = Literal.parseLiteral("target(" + target.name +
	// ",furniture)");
	// break;
	// case OBJECT:
	// target_belief = Literal.parseLiteral("target(" + target.name + ",object)");
	// break;
	// default:
	// target_belief = Literal.parseLiteral("target(" + target.name + ",unknown)");
	// break;
	// }

	// if (target.name != null) {
	// addPercept(target_belief);
	// }

	// }

}