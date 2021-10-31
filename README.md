# esp_mqtt_temperature_sensor

How it started:
![how it started](https://github.com/mattlazarowitz/esp_mqtt_temperature_sensor/blob/main/images/inital.jpg)
How it's going:
![How its going](https://github.com/mattlazarowitz/esp_mqtt_temperature_sensor/blob/main/images/current.jpg)

This is yet another project to create an IoT temperature sensor. 
I was trying to make something turnkey using an ESP-01 and a board with a regulatorand DHT11. There seemed to be plenty of examples out there to run this so I never planned to make a project out of it.

But then came the march of changes such as dealing with the fact that the DHT11 was getting heat soak from the regulator and the ESP-01, then replacing it entirely because it's humidity sensor is really bad (but the regulator on the board is surprisingly pretty good). Then there was the modding of the ESP-01 itself to try and take advantage of deep sleep because the original code and hardware was not suitable for being battery powered.

Then came the code changes. The code here is nearly entirely rewritten to use a different sensor, to use a model that leverages deep sleep, and has some error handling (though my error handling strategy is basically to turn it off and back on since none of the data is critical. 

The three things I did keep from the tutorial I started with is the MQTT broker, that I'm using Node Red as my means of seeing the information I'm capturing, and the use of the async MQTT library.

Besides the firmware, I'm including a sample group of nodes that can be imported into Node Red. This group can form a stand alone flow, or multiple copies can be placed into a flow (it won't interfere with copies of itself).

There is lots of work to do to make this cleaner and more usable to someone who is looking for something turnkey like I was. But I think I at least have something that has an OK error handling model and makes good use of the ESP8266's deep sleep capabilities. 

Sample Node Red node group:
![Sample Node Red node group](https://github.com/mattlazarowitz/esp_mqtt_temperature_sensor/blob/main/images/flow.JPG)

Note 1:
Error handling is pretty much "turn it off and back on again".
This works because the recovery action is generally to disconnect and try reconnecting again.
I drop the sample for that cycle because the data isn't really that important and I can tolerate dropped samples here and there.
For more systemic issues like an AP going offline, the system will use the SleepTimeArray[] to help preserve battery.
Note that this array can be customized but needs to contain at least one entry.
On successive timeouts, the system will work through this array up to the last value which it will reuse until it manages to successfully connect to wifi or it is power cycled.

A reconnect and a sleep-wake cycle look very similar so I just do a sleep-wake cycle. 
The time spent sleeping is variable and based on the number of successive retries. 
My values are pretty arbitrary but the idea is that the first 2 are shorter than a normal sleep-wake cycle. 
While the data isn't critical and it's OK to drop it, I still wanted to try and maintain the cadence as much as possible. 
Later entries in the sleep time array are longer because I assume there is an issue with network infrastructure that I'm working on and the ESP8266 should only check periodically to see if the issue is resolved.

I think there is still a bug with the longer delays because I have observed a sensor fail to come out of sleep at the expected time in a case where I was doing some infrastructure work. 
I need to set up a test AP and a test sensor to investigate further.


Note 2:
Like the overall error handling strategy, I keep things simple here. This is after some experimenting that resulted in some rather gnarly looking code.
What I observed with a combination of flaky AP and poor ESP8266 placement is that I could see the disconnect callback get invoked while connecting with a reason code that indicated the disconnect was due to an AP related issue.
I tried a recovery system where I would retry connecting to wifi in that specific case. But the results were inconsistent. Again, the "turn it off and turn it back on again" approach produced results as good as anything else.
Perhaps there would be more options if I tried working with the modem more directly but when I determined the root cause of the issues I was seeing, I doubt I could have done much in software anyway.


Note 3:
With the flaky AP and poorly positioned ESP8266, this could wait forever in the event that one or more topics fail to be acknowledged. 
I tried multiple recovery actions to get the enqueued topics published but in the end I continued with the overall strategy of one dropped update not being a huge issue.
The alternative I have been considering is an array or vector of topics with a flag that says they are published so I can have a system that tries to retransmit topics. 
Topics that were not transmitted could then be retransmitted after a recovery action. If recovery is handled via reboot, data could be stored in some nonvolatile location.
But if I took that route, I would want to make it so the topics are strings stored in an array that is held in flash so they could be referenced by index so that gets stored in nonvolatile memory is just the index value and the float data.

Note 4:
This somewhat odd looking system of 2 variables is something I arrived at after a bunch of experimenting.
The issue is that it's not clear what context the callbacks run in. There are no semaphores or locks in the standard Arduino core for the ESP8266 so I went with a producer-consumer which requires 2 variables since both loop() and mqttOnPublish() are both producers.
The system is working fine though I'd like to try using some type of atomic in the future. It looks like the ESP8266 core implements the standard C++ std::atomic operations. 
There appear to be other Arduino alternatives but using the standard C++ ones seems like the best idea. I'm saving this for a future 'rainy day' revision. 

TODOs:
1) Wrap the sensor specific code into a class. I've tried DHT11s, DHT22s, BMP280s, SHT30s, and DS18B20s. While my home setup is pretty homogeneous at this point, I think the wrapper would benefit someone else trying to make use of my code.
2) Create a version with the pub-sub library and see if there is an actual impact on the devices' battery endurance. 
3) Try using atomics from std::atomic for variables shared between the runtime code and the callbacks.
4) Add more quality of life features I've seen on other Arduino MQTT sensor projects and make it generally more product-like. For example, a way to set unique topics without compiling the Arduino project.
5) Improve the Node Red nodes in the logging section to combine the 2 function nodes into a single one that still allows for the nodes to be duplicated and only the MQTT nodes need to be updated to add a sensor to the mix.
6) Fix typos and errors in the Node Red nodes. The file has been thrown up here just to make it available for anyone who stumbles across this and want to use it
6) Make this readme prettier and include background on the physical construction of the sensor (as hackish as they are). Fix the errors that are in it.


