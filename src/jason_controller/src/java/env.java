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

    private Logger logger = Logger.getLogger("jason_controller."+env.class.getName());
    
    RosBridge bridge = new RosBridge();

    /** Called before the MAS execution with the args informed in .mas2j */
    @Override
    public void init(String[] args) {
        super.init(args);
		bridge.connect("ws://localhost:9090", true);
		logger.info("Environment started, connection with ROS established.");
    }

    @Override
    public boolean executeAction(String agName, Structure action) {

		switch(action.getFunctor()){
			case "test_ros_communication":
				test_ros_communication();
				break;
			case "at":
				logger.info("Moving to " + action.getTerm(0));
				break;
			default:
				logger.info("executing: "+action+", but not implemented!");
		}
		
		try {
            Thread.sleep(200);
        } catch (Exception e) {}
		
		updatePercepts();
        informAgsEnvironmentChanged();
        return true; // the action was executed with success
    }
    
	/** Tests Jason-ROS communication by publishing to the /jason/rotate topic, causing the 
		robot to briefly spin. */
	public void test_ros_communication() {
		logger.info("Testing ROS communication");
		Publisher pub = new Publisher("/jason/rotate", "std_msgs/Int16", bridge);
		Map<String,Integer> map=new HashMap<String,Integer>();  
  		map.put("data",5);  
		pub.publish(map);
	}

    /** creates the agents perception based on the MarsModel */
	void updatePercepts() {
        clearPercepts();
    }

    /** Called before the end of MAS execution */
    @Override
    public void stop() {
        super.stop();
    }
}
