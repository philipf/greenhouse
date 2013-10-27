import httplib
import json
import serial
import time
import string
import socket


apiEndPoint = "api.xively.com/v2/feeds/1575860722"
apiKey = "9TqjkWQznnIl3dMrNrhjm7mh4N9NjArJbFY9SEKDddVJjsZp"

print "Start"
print "API Endpoint: " + apiEndPoint
socket.setdefaulttimeout(10)

ser=serial.Serial("/dev/ttyACM0", 9600);
lastTime = time.time()
while True:
    try:
       arduinoData = ser.readline().rstrip('\r\n')
       print arduinoData
       timeToUpdate = time.time() - lastTime
       print "Time to update (seconds)", timeToUpdate
       data = {"version":"1.0.0"}
       if arduinoData[0:2] == "OK" and timeToUpdate >= 30:
          lastTime = time.time()
          readings = string.split(arduinoData[3:], ",")
          datastreams = []

          for r in readings:
             keyValue = r.split("=")
             sensorId = keyValue[0]
             value    = keyValue[1]
       
             measurement = {"id":sensorId, "current_value": value}
             datastreams.append(measurement)
 
          data["datastreams"] = datastreams
          payload = json.dumps(data, indent=4)
          print payload
          print "sending..."
          conn=httplib.HTTPConnection("api.xively.com")
          headers = {"X-ApiKey": apiKey}
          conn.request("PUT", "/v2/feeds/1575860722", payload, headers)
          response = conn.getresponse()
          print response.status, response.reason
          conn.close()
    
    except Exception as ex:
       print ex
       pass
   
    #time.sleep(60); # sleep for 5 seconds
