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
import javax.json.*;
import java.io.StringReader;
import java.util.StringTokenizer;

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

        private Command(JsonObject json){
            switch(json.getString("action")){
                case "fetch":
                    action = Action.SEARCH;
                    object = json.getString("object").toLowerCase();
                    accompanyType=AccompanyType.NULL;
                break;
                case "ACCOMPANY":
                    action = Action.ACCOMPANY;
                    object = json.getString("object").toLowerCase();
                    accompanyType = (json.getString("accompanyType").equals(AccompanyType.LEAD.toString())) ? AccompanyType.LEAD : AccompanyType.FOLLOW;
                break;
                case "MANIPULATE":
                    action = Action.MANIPULATE;
                    object = json.getString("object").toLowerCase();
                    location = json.getString("location");
                break;
            }
   
        }
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
        bdiEnvironment.logger.info("Getting commands");
        
        bdiEnvironment.publish("/hri/getGrannyAnnieRequest","std_msgs/String","");
        String jsonStringCmd = bdiEnvironment.subscribeSync("/hri/cloud_output", "std_msgs/String");

        String[] output = jsonStringCmd.split("},");
        for (int i=0;i<output.length;i++){
            output[i] = output[i].substring(1);
            output[i] += "}";
            
        }
        
        output[output.length-1]=output[output.length-1].substring(0, output[output.length-1].length() - 2);
        
        for (int i=0;i<output.length;i++){
            bdiEnvironment.logger.info(output[i]);
            JsonObject jsonCmd = bdiEnvironment.stringToJson(output[i]);
            Command cmd = new Command(jsonCmd);
            commands.push(cmd);
        }

        
    }



    public static void executeCommand(){
        bdiEnvironment.logger.info("Executing commands");
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
        //bdiEnvironment.pickup();
        bdiEnvironment.moveTo(location);
        //bdiEnvironment.place();
    }

    public static void search(String obj){
        bdiEnvironment.moveTo(obj);
        float[] xyz = bdiEnvironment.findObject(obj);
        bdiEnvironment.pickup(xyz);
        bdiEnvironment.moveTo("empty desk");
        bdiEnvironment.place();

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
