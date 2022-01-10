package webapp;

import com.google.gson.JsonObject;
import frc.robot.Robot;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map.Entry;

public class FireLog {
  private static HashMap<String, Object> topics = new HashMap<>();

  public static synchronized void log(String topic, Object value) {
    if(Robot.debug)
      topics.put(topic, value);
  }

  public static synchronized JsonObject toJsonObject() {
    JsonObject out = new JsonObject();
    
    Iterator<Entry<String, Object>> it = topics.entrySet().iterator();
    while ( it.hasNext() ) {
      Entry<String, Object> entry = it.next();
      Object value = entry.getValue();
      if ( value instanceof Number )
        out.addProperty(entry.getKey(), (Number)entry.getValue());
      else if ( value instanceof Double )
        out.addProperty(entry.getKey(), (Double)entry.getValue());
      else
        out.addProperty(entry.getKey(), entry.getValue().toString());
    }
    return out;
  }
}

