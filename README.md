# ROB538_project

For now, src/rob538Project.cpp is the main file that calls everything else. Start trying to understand the code there. All of the other files in /src are classes that hold the agents and the world as well as all of the agents tools for planning, their costmap, planner, ect. The jpgs are the different maps. Feel free to make your own using paint or by downloading, as long as the free values are white and obstacles are not, it should work.

To make it work you will need to have opencv installed and linked. This is because I use open cv mats to hold all of the data and visualize the simulator.

I will add comments throughout the code to try and explain what each step is doing, I tried to name the functions, classes, and variables as intuitively as possible, so hopefully it's legible. Let me know if you have any specific questions on what things do.

Some functionality I have developed but not included in this release is the ability to make a high level travel graph of the space and then plan over that. It significantly trims the search space when searching for options, I can add it if we want to, I didn't for now because that adds a few more classes and I felt like this was already an overwhelming amount to throw at you guys.

The current plan is to simply greedily explore the map, without conflict resolution (two agents can explore the same point) when/if I bring over the travel graph this will go away as agents will plan using prospective poses and information gain at those poses while accounting for other agent poses.


