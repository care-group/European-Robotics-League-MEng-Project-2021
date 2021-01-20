// Environment code for project hello_ros

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


public class RosEnv extends Environment {

    private Logger logger = Logger.getLogger("hello_ros."+RosEnv.class.getName());
    
    RosBridge bridge = new RosBridge();

    /** Called before the MAS execution with the args informed in .mas2j */
    @Override
    public void init(String[] args) {
        super.init(args);
		bridge.connect("ws://localhost:9090", true);
		logger.info("Environment started, connection with ROS established.");
		
		bridge.subscribe(SubscriptionRequestMsg.generate("/ros_to_java")
				.setType("std_msgs/String")
				.setThrottleRate(1)
				.setQueueLength(1),
			new RosListenDelegate() {

				public void receive(JsonNode data, String stringRep) {
					MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<PrimitiveMsg<String>>(PrimitiveMsg.class);
					PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
					logger.info(msg.data);
				}
			}
	);
    }

    @Override
    public boolean executeAction(String agName, Structure action) {
		if (action.getFunctor().equals("hello_ros")) {
			hello_ros();
		}
		else {
			logger.info("executing: "+action+", but not implemented!");
		}
        informAgsEnvironmentChanged();
        return true; // the action was executed with success
    }
    
	public void hello_ros() {
		System.out.println("Publishing..");
		Publisher pub = new Publisher("/jason/rotate", "std_msgs/Int16", bridge);
		Map<String,Integer> map=new HashMap<String,Integer>();  
  		map.put("data",5);  

		pub.publish(map);
	}

    /** Called before the end of MAS execution */
    @Override
    public void stop() {
        super.stop();
    }
}
