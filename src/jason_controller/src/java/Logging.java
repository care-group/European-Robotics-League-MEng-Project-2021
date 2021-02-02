import java.util.LinkedList;

import java.io.File; // Import the File class
import java.io.IOException; // Import the IOException class to handle errors
import java.io.FileWriter; // Import the FileWriter class
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;

public class Logging {

    public static class Change {
        private String name;
        private know_home.itemCategory category;

        public Change(String n, know_home.itemCategory c) {
            name = n;
            category = c;
        }
    }

    public static void createOutputDir(String file) {
        try {
            Path path = Paths.get(file).getParent();
            Files.createDirectories(path);
        } catch (IOException e) {
            System.err.println("Failed to create directory!" + e.getMessage());
        }

    }

    public static void createFile(String filename) {
        try {
            File myObj = new File(filename);
            myObj.createNewFile();
        } catch (IOException e) {
            System.err.println("An error occurred.");
        }
    }

    private static Map<know_home.itemCategory, LinkedList<String>> seperateCategories(LinkedList<Change> changes) {
        Map<know_home.itemCategory, LinkedList<String>> seperatedCategories = new HashMap<know_home.itemCategory, LinkedList<String>>();
        LinkedList<String> doors = new LinkedList<String>();
        LinkedList<String> furniture = new LinkedList<String>();

        for (Change change : changes) {
            switch (change.category) {
                case DOOR:
                    doors.push(change.name);
                    break;
                case FURNITURE:
                    furniture.push(change.name);
                default:
                    break;
            }
        }
        seperatedCategories.put(know_home.itemCategory.DOOR, doors);
        seperatedCategories.put(know_home.itemCategory.FURNITURE, furniture);
        return seperatedCategories;
    }

    public static void writeToFile(LinkedList<Change> changes, String filename) {

        Map<know_home.itemCategory, LinkedList<String>> seperatedChanges = seperateCategories(changes);
        try {
            FileWriter myWriter = new FileWriter(filename);
            myWriter.write("Door's closed:\n");
            for (String door : seperatedChanges.get(know_home.itemCategory.DOOR)) {
                myWriter.write("\t" + door + "\n");
            }

            myWriter.write("Furniture moved:\n");
            for (String furniture : seperatedChanges.get(know_home.itemCategory.FURNITURE)) {
                myWriter.write("\t" + furniture + "\n");
            }

            myWriter.close();
        } catch (IOException e) {
            System.err.println("An error occurred.");
        }

    }
}
