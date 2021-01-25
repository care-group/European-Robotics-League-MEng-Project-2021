import java.util.LinkedList;
import java.io.File; // Import the File class
import java.io.IOException; // Import the IOException class to handle errors
import java.io.FileWriter; // Import the FileWriter class
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class fileIO {
    public static void saveChanges(LinkedList<String> changes, String path) {
        createOutputDir(path);
        createFile(path);
        writeToFile(changes, path);

    }

    private static void createOutputDir(String file) {
        try {
            Path path = Paths.get(file).getParent();
            Files.createDirectories(path);
        } catch (IOException e) {
            System.err.println("Failed to create directory!" + e.getMessage());
        }

    }

    private static void createFile(String filename) {
        try {
            File myObj = new File(filename);
            myObj.createNewFile();
        } catch (IOException e) {
            System.err.println("An error occurred.");
        }
    }

    private static void writeToFile(LinkedList<String> changes, String filename) {
        try {
            FileWriter myWriter = new FileWriter(filename);
            myWriter.write("Door's closed:\n");
            for (String change : changes) {
                myWriter.write("\t" + change + "\n");
            }
            // myWriter.write("Files in Java may be tricky, but it is fun enough!");
            myWriter.close();
        } catch (IOException e) {
            System.err.println("An error occurred.");
        }

    }
}
