# Claude instructions for ros2sysmon 
    * Code is python
    * Uses only ROS2
    * functions and methods no longer than 50 lines
    * Write idiomatic python
    * Look for libraries that exist instead of reinventing
    * Don't go overboard on error checking
    * Give methods intention revealing names
    * Use classes and put them in a seprate file
    * Put data classes in the file where they are constructed
    * name files after the class defined in the file
    * no files with more than about 300 lines
    * use python latest and ros2 compliant package management and building with colcon
    * prefer async/await over threading when there is a choice
    * avoid if/else statements that are nested more than 1 deep
    * avoid 1 line methods
    * look for code duplication and make the code DRY if it makes sense
    * When undertaking a multi step implemenation or refactoring, do it in a way that after each step we retain a running program so that I can test it to make sure we are on the right track.
    * Don't assign the result of a function to a variable just to use that variable one time only, Just use the function call.
    * Avoid simple wrappers
    * Avoid 1 line functions and methods
    * Put a comment with each method or function ONLY if the name of the function is not sufficient by itself.
    * Almost always use double quotes. Only use single quotes if there is a specific requirement to do so
    

# CURRENT.md
    * Now read CURRENT.md and follow it's instructions
