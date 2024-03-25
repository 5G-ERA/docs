# Non-ROS Net-App tutorial 

This tutorial mainly focuses on how to create Non-ROS network applications. 

## Requirements :
* Install the [era_5g_interface](https://github.com/5G-ERA/era-5g-interface) and [era_5g_client](https://github.com/5G-ERA/era-5g-client)
* pip install era_5g_interface 
* pip install era_5g_client
* [Python 3](https://www.python.org/downloads/)  installations is required for example 
* [Docker](https://docs.docker.com/compose/install/) installations. 
* [Visual studio code](https://code.visualstudio.com/download) to run the package. 

Furthermore, the requirements could be found here; [Reference-Netapp](https://github.com/5G-ERA/Reference-NetApp) 

## Steps for the tutorial : 
* Step 1: Using the package era_5g_network_application_template

* Step 2: Copy the package and create a new package for net-app (adding_two_numbers) and change the name of the directory of source_code in the directory as well. 

* Step 3: Change settings in the python package file and the name 

![image](../Tutorials/imgs/pythonname.png)


* Step 4: Create client.py file 

* Step 5: change the name and package in setup.py file as well and make sure the requirements are up-to-date as well. 

![image](../Tutorials/imgs/setupname.png)

* Step 6: After setup.py and requirements, the next step is to follow the interface.py file to see the main end points of the net app i.e register and connect. 
JSON callback http method : rest endpoint [post] :this endpoint meant to check the processing json from the client
The netapp checks if the robot or the client who calls this endpoint is registered and after that it uses the session ID to select the appropriate task handler shown below: 
It will send the data obtained from the client back to the client so the netapp (to add two numbers), change the behavior. 

![image](../Tutorials//imgs/approute.png)

* Step 7 : Add data n1 and n2 
![image](../Tutorials/imgs/n1andn2.png)

* Step 8: Next step is to run the netapp. 
Type these commands to run the environment 
In order to run it, we need to create a virtual environments and install all the necessary requirements ; first activate the environment 
pip install -r requirement.txt
pip install -e .

* Step 9: Check that the file is running by using the commands, 
cd era_5g_adding_two_numbers
python interface.py
You will see its running; 

![image](../Tutorials/imgs/running.png)

* Step 10: To be able to run the web server, we need a simple client, so to create client.py 
First activate source and pip install era_5g_client

* Step 11: Callback function 
We need callback function to get the results, make a client.py file, make sure to add the address of the callback function 

![image](../Tutorials/imgs/Callbackfunc.png)










