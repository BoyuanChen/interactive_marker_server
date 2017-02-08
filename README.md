To run the interactive marker server:
```
rosrun interactive_marker_server interactive_marker_server.py 
```
To run the robot web tools at the same time:
```
roslaunch system_launch webui.launch
```
To bring it up to the browser: run the commands below in your js file directory
```
python -m SimpleHTTPServer 8081
```
To run the real Graspit! in the backend:
```
roslaunch sytem_launch fetch_basestation.launch
```

