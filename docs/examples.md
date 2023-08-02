Examples
===

---

## Code Samples

There is sample code under the Code Samples directory.

These are considered part of the documentation and not built by default.


### robot.unittestable

The unittestable robot is an example of a robot that supports unit testing.

It contains a single subsystem called "ExampleRail" that implements a few commands.
The rail consits of a motor with a builtin encoder and a pair of limit switches.
Presumably the motors are driving something along a path segment that is terminated
on each end by a limit switches.

The commands move control the movement to the designated end or mid point.

The tests show how the construction of the robot objects and the execution of the
command can be tested using unit tests without having a physical robot.


#### Running the test

Since the example code is not compiled by default, you will need to modify
the build.gradle script (in your local workspace only) to build the samples.

```
sourceSets.main.java.srcDirs += ['CodeSamples/src/main/java']
sourceSets.test.java.srcDirs += ['CodeSamples/src/test/java']
```

Once this is added, the tests should be visible to both gradle and visual code
where you can run them as any other.

If you want to run this in the WPILib simulator, you will need to change the
default robot in the build.gradle. **Do not check it in to the main branch**
or it will change the robot being deployed to this sample rather than the
repository's real Cougar Robot.

The change would be
```
def ROBOT_MAIN_CLASS = "team1403.examples.robot.unittestable.Main"
```
