# Welcome to the Robot Path Planning Static Library!

![image](https://user-images.githubusercontent.com/77941062/226486518-0254b83f-2332-4adf-aaf4-e6388d5aed5a.png)


## Solution & Methodology
### Class Diagram
![ClassDiagram1](https://user-images.githubusercontent.com/77941062/226496178-a1d9415a-e8e0-40db-b76f-9471426929e0.png)

## Example Application
### The Set Up 
#### Get Instant Access to the Library and source Files 
1. Open Visual Studio Code 2019
2. Navigate to Clone a Repository
3. Enter the Git Repository URL

#### Manually create a C++ console app
1. Open Visual Studio Code 2019
2. Navigate to Create a new project
3. At the top of the dialog, set the Project type filter to C++, Windows and Console.
4. From the filtered list of project types, choose Console App then choose Next.
5. Choose a name for your console app. ‘RPPClient’ Works
6. Choose the Create button to create the client project.
7. Open the shortcut menu for the RPPClient project in Solution Explorer, and then choose Add > Reference.
8. Open the Projects tab, select the RPPLibrary check box, and then choose the OK button.
9. In Solution Explorer, right-click on RPPClient to open the shortcut menu. Choose Properties to open the RPPClient Property Pages dialog box.
10. Open Linker -> Input and browse for the RPPLibrary.lib Object File Library
11. In the RPPClient Property Pages dialog box, set the Configuration drop-down to All Configurations. Set the Platform drop-down to All Platforms.
12. Select the Configuration Properties > C/C++ > General property page. In the Additional Include Directories property, specify the path of the RPPLibrary directory (can be downloaded from git), or browse for it.
13. Open the Additional Include Directories property value drop-down list, and then choose Edit.
14. In the Additional Include Directories dialog box, double-click in the top of the text box. Then choose the ellipsis button (...) at the end of the line.
15. In the Select Directory dialog box, navigate up a level, and then select the RPPLibrary directory. Then choose the Select Folder button to save your selection.
16. In the Additional Include Directories dialog box, choose the OK button.
17. In the Property Pages dialog box, choose the OK button to save your changes to the project.
18. You can now use the RPPLibrary in this app by including the #include "RobotAlgo.h" and #include "RobotMap.h"

### Usage
#### Run the RPPClient.exe for a Hands on Demo only on windows
https://github.com/TarekElDick/StaticRPP/blob/master/RPPClient.exe
#### Or if you improted the lib into your project
![image](https://user-images.githubusercontent.com/77941062/226737445-5b12ab86-f600-4978-988a-05aca6cb0f28.png)

## Implementation
## A Star Vs Dijkstra’s algorithm
A* algorithm uses heuristics.
Faster and more efficient than Dijkstra's algorithm. 
Handles problems with obstacles and varying terrain costs more effectively than Dijkstra's algorithm. 
However, A* algorithm requires more computational resources and memory compared to Dijkstra's algorithm.

## Limitations
### Validation
While the code contains some validation checks, it could be improved to validate input parameters more thoroughly. 

For example, the algorithm assumes that the start and end nodes are not touching obstacle nodes, but it does not explicitly check for this. 

Additionally, the code could validate that the robot radius is within the bounds of the map and is not too large for the map size.

### Modularity
The code could be made more modular by separating the algorithm's logic from the visualizer logic. 

This would allow the algorithm to be used without the visualizer or with different visualizers.

### Error Handling
The error handling in the code could be improved.

For example, if an exception is thrown during the algorithm's execution, the program will terminate abruptly. 

A better approach would be to handle the exception gracefully and provide a meaningful error message to the user.

### Robot Movement
The algorithm requires the ability to search through nodes that have already been visited, which is achieved by allowing the robot to move back to nodes on the closed list.

Although this movement is not realistic, it is necessary for the algorithm to function correctly.


## Performance Improvements to do
Currently Heuristics are pre calculated before we run the algorithm. 

New improvement calculate Admissible Heuristic as the robot is moving.

Saves on computational resources but might cause robot to take extra steps.

Robot might travel deeper even though it will hit an obstacle. 

Heuristics become admissible instead of exact.

Paralyzing loops

Avoid redundant calculations

Use a priority queue for open list, instead of a vector,

Use more const references

