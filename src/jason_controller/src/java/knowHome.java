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

public class knowHome extends Environment {

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

    public static class Target {
        public String name;
        public itemCategory type;

        private Target() {
            name = null;
            type = itemCategory.UNKNOWN;
        }

        public void update(String n, itemCategory category) {
            name = n;
            type = category;
        }

        public boolean targetIs(itemCategory category) {
            return (type == category);
        }

        public boolean isEmpty() {
            return name == null;
        }

    }

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

    public static boolean checksComplete() {
        return (rooms.isEmpty());
    }

    public static boolean doorChecksComplete() {
        if (rooms.isEmpty()) { // prevents out of bounds error when called if rooms is empty
            return true;
        } else {
            return (rooms.getFirst().doors.isEmpty());
        }
    }

    public static boolean furnitureChecksComplete() {
        if (rooms.isEmpty()) { // prevents out of bounds error when called if rooms is empty
            return true;
        } else {
            return (rooms.getFirst().furniture.isEmpty());
        }
    }

    // Only to be 2 moved objects, once we have found 2 moved objects, can assume
    // this is complete, letting us end object checks early.
    public static boolean objectChecksComplete() {
        return (movedObjectFoundCounter >= 2);
    }

    public static boolean getChangeDetected() {
        return changeDetected;
    }

    public static void setChangeDetected(boolean b) {
        changeDetected = b;
    }

    public static Target getTarget() {
        return target;
    }

    public static void saveChanges() {
        String path = "/workspace/output/changes.txt";
        Logging.createOutputDir(path);
        Logging.createFile(path);
        Logging.writeToFile(changes, path);
    }

    public static void next(String objCategory) throws Exception {
        try {
            switch (objCategory) {
                case "room":
                    rooms.removeFirst();
                    target.update(rooms.getFirst().roomName, itemCategory.ROOM);
                    break;
                case "door":
                    target.update(rooms.getFirst().doors.getFirst(), itemCategory.DOOR);
                    break;
                case "furniture":
                    target.update(rooms.getFirst().furniture.getFirst(), itemCategory.FURNITURE);
                    break;
                case "object":
                    Map.Entry<String, String> firstEntry = objects.entrySet().iterator().next();
                    String location = firstEntry.getKey();
                    target.update(location, itemCategory.OBJECT);
                    break;
                default:
                    bdiEnvironment.logger.info("Unsure what to iterate through!");
                    target.type = itemCategory.UNKNOWN;
                    return;
            }
        } catch (Exception e) {
        }
    }

    public static void printRooms() {
        for (Room room : rooms) {
            bdiEnvironment.logger.info(room.roomName);
            bdiEnvironment.logger.info("\tDoors");
            for (String door : room.doors) {
                bdiEnvironment.logger.info("\t\t" + door);
            }
            bdiEnvironment.logger.info("\tFurniture");

            for (String f : room.furniture) {
                bdiEnvironment.logger.info("\t\t" + f);
            }
        }
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
        bdiEnvironment.logger.info("Inspecting " + item);
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

    public static LinkedList<Literal> getKnowMyHomePercepts() {
        LinkedList<Literal> percepts = new LinkedList<Literal>();

        if (checksComplete()) {
            percepts.add(Literal.parseLiteral("done(rooms)"));
        }
        if (doorChecksComplete()) {
            percepts.add(Literal.parseLiteral("done(doors)"));
        }
        if (furnitureChecksComplete()) {
            percepts.add(Literal.parseLiteral("done(furniture)"));
        }
        if (objectChecksComplete()) {
            percepts.add(Literal.parseLiteral("done(objects)"));
        }

        if (getChangeDetected() && getTarget().targetIs(itemCategory.DOOR)) {
            percepts.add(Literal.parseLiteral("closed"));
            setChangeDetected(false);

        } else if (getChangeDetected()
                && (getTarget().targetIs(itemCategory.FURNITURE) || getTarget().targetIs(itemCategory.OBJECT))) {
            percepts.add(Literal.parseLiteral("moved(" + getTarget().name + ")"));
            setChangeDetected(false);
        }

        Literal targetLiteral;
        switch (getTarget().type) {
            case DOOR:
                targetLiteral = Literal.parseLiteral("target(" + getTarget().name + ",door)");
                break;
            case ROOM:
                targetLiteral = Literal.parseLiteral("target(" + getTarget().name + ",room)");
                break;
            case FURNITURE:
                targetLiteral = Literal.parseLiteral("target(" + getTarget().name + ",furniture)");
                break;
            case OBJECT:
                targetLiteral = Literal.parseLiteral("target(" + getTarget().name + ",object)");
                break;
            default:
                targetLiteral = Literal.parseLiteral("target(" + getTarget().name + ",unknown)");
                break;
        }

        if (!getTarget().isEmpty()) {
            percepts.add(targetLiteral);
        }

        return percepts;

    }

    private static void inspectDoor(String door) {
        // stub code that will be replaced by a topic subscriber that informs of the
        // door's position
        Random r = new Random();
        changeDetected = r.nextBoolean();

        bdiEnvironment.logger.info(door + " is " + (changeDetected ? "closed" : "open"));
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

        bdiEnvironment.logger.info(furniture + (changeDetected ? " has moved" : " remains untouched"));
        if (!changeDetected) {
            rooms.getFirst().furniture.removeFirst(); // remove furniture as we no longer care about it once
                                                      // confirmed it's found
        } else {
            changes.push(new Logging.Change(furniture, itemCategory.FURNITURE));
        }
    }

    public static void openDoor() {
        bdiEnvironment.logger.info("Opening door.");
        changeDetected = false;
        rooms.getFirst().doors.removeFirst(); // remove door as we no longer care
                                              // about it after opening it.

    }

    public static void find(String item) {
        if (target.type == itemCategory.FURNITURE) {
            bdiEnvironment.logger.info("finding " + item);
            changeDetected = false;
            rooms.getFirst().furniture.removeFirst(); // remove furniture as we no longer care about it
                                                      // after finding it
        } else {
            bdiEnvironment.logger.info("finding " + objects.get(item) + " at other furniture");
            changeDetected = false;
            movedObjectFoundCounter++;
            Map.Entry<String, String> firstEntry = objects.entrySet().iterator().next();
            objects.remove(firstEntry.getKey());
        }

    }

    private static void lookForObjectOnFurniture(String furniture) {
        String object = objects.get(furniture);
        bdiEnvironment.logger.info("Scanning " + furniture + " for " + object);

        // stub code that will be replaced by a topic subscriber that informs if the
        // object is detected
        Random r = new Random();
        changeDetected = r.nextBoolean();
        bdiEnvironment.logger.info(object + (changeDetected ? " has moved" : " has been found at " + furniture));
        if (!changeDetected) {
            objects.remove(furniture); // remove object as we no longer care about it once
                                       // confirmed it's found
        } else {
            changes.push(new Logging.Change(object, itemCategory.OBJECT));
        }
    }

}
