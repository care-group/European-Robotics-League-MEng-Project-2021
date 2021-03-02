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

public class catering extends Environment {


    public static enum Action {
        SEARCH, ACCOMPANY, MANIPULATE, NULL
    };

    public static enum AccompanyType {
        LEAD, FOLLOW, NULL
    };

    public static class Command{
        private Action action;  // eg. get
        private String object;  // eg. coffee
        private String location; //eg. kitchen
        private AccompanyType accompanyType;
        private Command(Action a, String o){
            action = a;
            object = o;
            accompanyType=AccompanyType.NULL;
        }
        
        //Accompany constructor
        private Command(Action a, String o, AccompanyType at){
            action = a;
            object = o;
            accompanyType=at;
        }

        //Manipulate constructor
        private Command(Action a, String o, String l){
            action = a;
            object = o;
            accompanyType=AccompanyType.NULL;
            location=l;
        }
    }
    public static LinkedList<Command> commands = new LinkedList<Command>();
    private static LinkedList<String> likelyLocations = new LinkedList<String>(Arrays.asList("kitchen_table","kitchen_counter","kitchen_cupboard"));
   
    public static void getCommand(){
        //pub/sub to NLP stack.
        //populate commands#
        bdiEnvironment.logger.info("Getting commands");
        Command c1 = new Command(Action.SEARCH,"bottle");
        Command c2 = new Command(Action.ACCOMPANY,"kitchen",AccompanyType.FOLLOW);
        Command c3 = new Command(Action.ACCOMPANY,"bathroom",AccompanyType.LEAD);
        Command c4 = new Command(Action.MANIPULATE,"bottle","kitchen_table");
        
        commands.push(c4);
        commands.push(c3);
        commands.push(c2);
        commands.push(c1);

    }

    public static void executeCommand(){
        for(Command cmd : commands){
            bdiEnvironment.logger.info("Executing command "+cmd.action.toString());
            switch (cmd.action) {
                case SEARCH:
                    search(cmd.object);
                    break;
                case ACCOMPANY:
                    accompany(cmd.object,cmd.accompanyType);
                    break;
                case MANIPULATE:
                    manipulate(cmd.object,cmd.location);
                    break;
                default:
                    bdiEnvironment.logger.info("Unknown command action...");
                    break;
            }
        }
    }
    private static void accompany(String obj, AccompanyType accompanyType){
        switch (accompanyType) {
            case LEAD:
                bdiEnvironment.escort(obj);
                break;
            case FOLLOW:
                bdiEnvironment.follow(obj);
                break;
            default:
                break;
        }

    }
    private static void manipulate(String obj, String location){
        bdiEnvironment.moveTo(obj);
        bdiEnvironment.pickup();
        bdiEnvironment.moveTo(location);
        bdiEnvironment.place();

    }

    private static void search(String obj){
        String foundLocation = "";
        for(String loc : likelyLocations){
            bdiEnvironment.moveTo(loc);
            boolean found = find(obj);
            if(found){
                foundLocation=loc;    
                break;
            }
        }
        bdiEnvironment.logger.info("Found at "+ foundLocation);

    }

    private static boolean find(String object){
        bdiEnvironment.logger.info("Finding obj");
        return true;
    }

    public static LinkedList<Literal> getCateringLiterals() {

        LinkedList<Literal> percepts = new LinkedList<Literal>();
        // if(commands.isEmpty()){
        //     percepts.push(Literal.parseLiteral("done(commands)"));
        //     bdiEnvironment.logger.info("done!");
        // }

        return percepts;
    }
}
