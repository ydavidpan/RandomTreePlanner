This code is distributed as part of the hands-on demonstration in class for Thursday 9/20/2016.  This readme includes instructions on how to setup the included code in preparation for the hands-on lecture.  Your code from project 2 is necessary to make it work.

If you are using the VM distributed for the class then all of the following should get the code working.  If you are not then you may need to alter the Makefile slightly for it to work on your platform.  The Makefile includes notes where such changes might need to be made.

1. Install python-matplotlib if it is not already installed.
    $ sudo apt-get install python-matplotlib
2. Move the included code into a working directory along with the code you wrote for project 2 and the header file included with project 2:
    -CollisionChecking.h
    -CollisionChecking.cpp
3. From the working directory execute make:
    $ make
4.  You are now ready to use the code.  You can test the planner from the working directory with:
    $ ./MyRigidBodyPlanning
5.  You will be asked which robot you would like to plan for:
Plan for: 
 (1) A point in 2D
 (2) A rigid box in 2D
 (3) Mystery!

6.  Choose either 1 or 2 which will create a path.txt file.
    We will get to 3 in class!
7.  Execute plotSolution.py.
    $ ./plotSolution.py
8. This should show a visualization of the solution.  If plotSolution.py fails to run try instead running python plotSolution.py.
