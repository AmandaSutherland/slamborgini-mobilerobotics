slamborgini-mobilerobotics
==========================

For the Mobile Robotics project in CompRobo. 

"""
October 3, 2014 - Adjustments were made to Paul's particle filter code 
to gather all the statements which interact with gazebo, rviz, or 
various rostopics.  The rest is a pseudo code structure.

October 4, 2014 - C+V: Finished formatting base code form Paul's inclass example (with some minor adjustments and renamings, and from pf_code that seemed useful, especially the transforms). Tested code - results was: it worked!

October 6, 2014 - C+A: Reviewed code so far, attempted to implement on computers.  Serious issues with launching it on their computers.

October 7, 2014 - All: In class we got everyone's workspace up and running.  Then we wrote pseudocode to implement at a meeting to be held later today.  At that next meeting we discussed the use of Bayes.  We decided to keep the current bayes set-up, and played with the spectrum.  We have a plan on logging data into a database once it is read, and as it fades the new color of the database log will come through.  After this, we will work on robot localization, by setting up a simple particle filter (maybe) and relate it to our generated map.

October 10, 2014 - All: In class Amanda worked on getting dynamic obstacles develped in Gazebo.  Claire and Victoria worked on adjusting the map display, and identifying the spaces in which Bayesian updates should be made and developed.  In the evening we played with the bayesian equations and attempted to make a quickly updating map.  This was achieved by updating the hits within the particle for loop, and limiting how 'unlikely' something was with being present.  By doing this, you can create the effect of a more dynamic environment, by assuming some arbitrarily small liklihood that SOMETHING will be in a location, no matter how many times you read it as blank.

October 11, 2014 - C+A: Added dynamic obstacles and second map that has only them plotted on it. Plots on orginal map as well. Obstacles dissapear after 15 cycles.

October 12, 2014 - All: We made the thingy work!  Mapping is still rough, can optimize tomorrow.  Adjusted things like top cap of odds, constants, adjusting pathing, etc.

October 13, 2014 - All: Finished it up!
"""
