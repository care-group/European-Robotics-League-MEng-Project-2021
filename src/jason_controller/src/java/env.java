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
		LinkedList<String> doors;

		private Room(String n, LinkedList<String> d) {
			this.roomName = n;
			this.doors = (LinkedList<String>) d.clone();
		}
	}

	private Logger logger = Logger.getLogger("jason_controller." + env.class.getName());
	RosBridge bridge = new RosBridge();

	public static LinkedList<Room> rooms = new LinkedList<Room>();
	public static LinkedList<String> changes = new LinkedList<String>();

	public static String target_location;
	public static boolean isTargetRoom = true;
	public static boolean doorClosed = false;

	/** Called before the MAS execution with the args informed in .mas2j */
	@Override
	public void init(String[] args) {
		super.init(args);
		// bridge.connect("ws://localhost:9090", true);
		logger.info("Environment started, connection with ROS established.");

		initRooms();
		printRooms();
		updatePercepts();
	}

	public void initRooms() {
		Room r1 = new Room("living_room", new LinkedList<String>(Arrays.asList("door1", "door1.2", "door1.3")));
		Room r2 = new Room("kitchen", new LinkedList<String>(Arrays.asList("door2.1", "door2.2")));
		Room r3 = new Room("bedroom", new LinkedList<String>(Arrays.asList("door3.1")));

		rooms.add(r1);
		rooms.add(r2);
		rooms.add(r3);

	}

	@Override
	public boolean executeAction(String agName, Structure action) {
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
					logger.info("Inspecting " + action.getTerm(0));

					// stub code that will be replaced by a topic subscriber that informs of the
					// door's position
					Random r = new Random();
					doorClosed = r.nextBoolean();
					logger.info(action.getTerm(0) + " is " + (doorClosed ? "closed" : "open"));

					if (doorClosed) {
						changes.push(action.getTerm(0).toString());
					}

					rooms.getFirst().doors.removeFirst(); // remove door as we no longer care about it after inspecting.
					break;
				case "open":
					logger.info("Opening door.");
					doorClosed = false;

					break;
				case "next":
					iterate(action.getTerm(0).toString());
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

	void printRooms() {
		for (Room room : rooms) {
			logger.info(room.roomName);
			for (String door : room.doors) {
				logger.info('\t' + door);
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

	void move_to(String location) throws Exception {
	}

	void iterate(String objCategory) throws Exception {
		try {
			switch (objCategory) {
				case "room":
					rooms.removeFirst();
					isTargetRoom = true;
					target_location = rooms.getFirst().roomName;
					break;
				case "door":
					isTargetRoom = false;
					target_location = rooms.getFirst().doors.getFirst();
					break;
				default:
					logger.info("Unsure what to iterate through!");
					isTargetRoom = false;
					return;
			}
		} catch (Exception e) {
		}
	}

	/** creates the agents perception based on the MarsModel */
	void updatePercepts() {
		clearPercepts();

		Literal target = Literal
				.parseLiteral("target(" + target_location + "," + (isTargetRoom ? "room" : "door") + ")");

		if (checksComplete()) {
			addPercept(Literal.parseLiteral("done(rooms)"));
		} else if (doorChecksComplete()) {
			addPercept(Literal.parseLiteral("done(doors)"));
		}
		if (target_location != null) {
			addPercept(target);
		}

		if (doorClosed) {
			logger.info("doorClosed boolean is true");
			addPercept(Literal.parseLiteral("closed"));
		}

	}

	/** Called before the end of MAS execution */
	@Override
	public void stop() {
		super.stop();
	}
}
