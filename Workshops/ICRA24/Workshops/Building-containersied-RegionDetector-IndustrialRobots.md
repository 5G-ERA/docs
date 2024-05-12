## Building a Containerised, Portable Region Detection service for Industrial Robots

This demonstration is designed to firstly provide context and applications for a computer vision based region detector in an industrial setting, before moving on to walking you through the steps to build and deploy the region detector yourself, ahead of the later hands-on session.

Hopefully by the end of this presentation you're able to appreciate the value of developing containerised robotics software solutions in industrial scenarios, and are ready to dive in and try to deploy your own region detector.

### 1) Demonstration: Motivation and running the region detector for an Industrial Robot Simulation.

This first section of the demonstration will go start by introducing a use case where we at HAL Robotics are actively using the technology showcased later in the session in a real world application. 

This section of the presentation will start with a [video](https://drive.google.com/file/d/1qmtNsb_9QID7kQLCN4q-YTQn3BLbD1N-/view?usp=drive_link) showing how computer-vision based region detection enables rapidly reconfigurable, human-led robotics applictions. 

This is then followed by a live demonstration of Decode by HAL Robotics, showcasing the region detection service running in a simulation of the same cell.

We then transition over to introducing the audience to the repository, which can be found [here](https://github.com/5G-ERA/region-detection). This then moves us on to the second portion of the demonstration.

### 2) Setting up and deploying the region detector.

#### 2.1 Requirements:
The requirements for this portion of the Tutorial are designed to reduce the barrier to entry as much as possible. Nevertheless, to participate you will need the following installed on your personal device:
- Python (to at least version 3.11.9)
- Docker (We will use Docker desktop on Windows 11.)
- An IDE of your choice (for the demonstration we will use VS Code.)
- A stable internet connection

Note that this demonstration assumes some familiarity with the Python programming language, and at least a basic understanding of the core concepts behind Docker, but does not require adept skill in either.

Further, this demonstration does not specify a choice of operating system, and has been tested on both Windows 11 and Ubuntu 22.04, however if issues in this regard are noticed during the tutorial we will do our best to support.

#### 2.2 Setting up

Firstly, please download the github repository for this demo. We recommend doing this by:

```bash
git clone https://github.com/5G-ERA/region-detection.git
```

Now, inside your IDE of choice, navigate to the root directory of the project.

You should see the following files and sub-directories:
```
- __region_detector.py__
- __icra_tutorial.ipynb__
- requirements.txt
- README.md
- data/
- proto/
- grpc-compiled/
```
The only parts of this repository you are required to interact with in order to complete this tutorial are the first two files, the __region-detector__ python script, and the accompanying __icra-tutorial__ jupyter notebook.

#### 3) Demonstration Task

Your task for this demonstration is to finish the provided region detection service such that it is capable of correctly identifying the __complete closed regions__ in the provided images. For example, a successful result on image ``'test3.png'`` should look like this:

[image](../Workshops/Imgs/region_detection.png)

For final submission, you should make your changes to the file __region_detector.py__. If you are not comfortable writing your own function for this task, or do not desire to do so, fine tuning the parameters for minimum edge count, precision, and binary threshold value is sufficient to complete the activity.

If you chose to write your own function, we advise adding it to the __region-detector.py__ file at line 78. You should then replace line 119 such that your new function is called. For reference, line 119 should originally read:
```python
polylines, vertices = self.find_regions(image.copy(), min_edges=MIN_EDGES, precision=PRECISION, binaryThreshold=THRESHOLD) 
```

We advise you to use the provided jupyter notebook to play with the region detection service with visual feedback to understand it. 

Good luck and enjoy!

Paul McHard