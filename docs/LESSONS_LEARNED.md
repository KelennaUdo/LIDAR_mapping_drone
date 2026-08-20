# Lessons Learned

This project has involved more than building a drone simulation. It has also
required me to decide which problems I actually want to solve, which tools I
need to understand, and which existing systems I should reuse. These are the
most important lessons I have learned so far.

## 1. Narrow the Project Before Building

My original idea of making a robotics project was too broad. Robotics includes
many different platforms, problems, and areas of study, so I first needed to
choose a specific form factor. I decided to focus on a multicopter drone.

This gave the project a clearer boundary. Instead of trying to learn every part
of robotics at once, I could concentrate on flight, sensing, mapping, and
autonomous navigation within one platform.

**Lesson:** A project becomes easier to reason about when its physical platform
and main purpose are defined early.

## 2. Decide What I Actually Want to Design

I initially imagined this project as a complete design challenge. My plan was
to:

- design the drone frame from scratch;
- manufacture it with a 3D printer; and
- select and purchase all the required electronic components.

I soon realized that this would turn the project into a hardware-development
project. Building a reliable physical drone would take a great deal of time,
and it was not the part of robotics that interested me most. It would also make
experimentation slower and more expensive.

I therefore chose to begin with a simulated drone in Gazebo. This allowed me to
focus on the software questions that interested me: control, sensing, ROS 2,
autonomy, and mapping.

**Lesson:** Building every component from scratch is not automatically more
valuable. The implementation scope should support the questions the project is
trying to answer.

## 3. Simulation Needs the Right Computing Environment

I originally tried to run the project in an Ubuntu virtual machine using Oracle
VirtualBox. However, the virtual machine could not use my graphics card
effectively for Gazebo rendering, so simulation performance was poor.

I eventually installed Ubuntu directly alongside Windows in a dual-boot setup.
This took time and introduced a few minor system issues, especially because I
was learning the process through ChatGPT, YouTube videos, and experimentation.
Despite that difficult transition, native Ubuntu has worked much better for the
project.

**Lesson:** A virtual machine may be convenient for ordinary development, but
graphics-heavy robotics simulation often benefits from native GPU and system
access. The development environment can determine whether the rest of the
project is practical.

## 4. Reuse Models When Modeling Is Not the Research Question

After deciding not to build the physical drone, I thought I should at least
create its complete software model myself. That also proved more complicated
than expected. A useful simulation model involves several connected concepts,
including:

- SDF model structure;
- links, joints, collision geometry, and visual geometry;
- URDF and `robot_description`;
- TF frame relationships; and
- RViz visualization.

These were all valuable concepts to learn, but creating a complete vehicle
model was not the main question I wanted to investigate. I therefore adopted an
existing Gazebo vehicle, first the X3 and later PX4's supported X500. I could
then modify the model only where the project required it, such as adding LiDAR
sensors.

**Lesson:** Reusing a proven model leaves more time for the parts of the system
that are actually being studied. Existing components can still be examined and
extended without rebuilding them completely.

## 5. Learn From a Custom Controller, Then Use PX4

I also had to decide whether to build my own multicopter controller or use an
established flight-control system. I did not skip the custom-controller path. I
watched Brian Douglas's MATLAB series on multicopter control and implemented a
controller for the X3 simulation.

The controller flew, which taught me a great deal about cascaded control loops,
PID gains, motor mixing, state feedback, and safety limits. However, it did not
fly reliably enough to support the larger mapping project. Tuning the connected
PID loops was difficult, and the vehicle continued to suffer from position
drift.

I chose PX4 for the main X500 platform because it already provides mature
attitude control, position control, state estimation, motor mixing, and flight
safety behavior. The original X3 controller remains useful as a learning
sandbox rather than being discarded as a failed attempt.

**Lesson:** A from-scratch implementation can be extremely valuable for
learning even when it is not the final production solution. After understanding
the fundamentals, using a mature system can move the larger project forward.

## 6. Use Docker to Control Version Complexity

My host computer runs Ubuntu 26.04 and ROS 2 Lyrical, while the selected PX4
toolchain is better supported in an Ubuntu 24.04 environment. Docker gave me a
way to run that compatible environment without replacing my host operating
system or mixing every PX4 dependency into it.

Docker became another substantial topic that I had to pause and learn. Images,
containers, bind mounts, graphics access, and persistent storage were initially
confusing. The setup also consumes several gigabytes of disk space. Despite
those costs, it has provided a repeatable PX4 environment and reduced the risk
of dependency conflicts.

**Lesson:** Isolation tools introduce their own complexity, but that complexity
can be worthwhile when a project depends on software with different operating
system and library requirements.

## 7. Use AI as a Learning and Development Partner

I can read and write C++ and Python at an intermediate level, but this project
combines many large subjects: ROS 2, Gazebo, PX4, RViz, SLAM, multicopter
dynamics, autonomous navigation, sensor fusion, Linux, Docker, and networking.
It was difficult to plan the overall architecture, learn the underlying
concepts, debug the environment, and implement every detail in code at the same
time.

I therefore chose to use AI extensively for coding assistance and project
guidance. This allowed me to spend more time understanding how the complete
system fits together while still making implementation progress. It was most
useful when the work was divided into small checkpoints and each command,
component, and code change was explained before being used.

The goal is not to avoid understanding the generated code. The goal is to use
AI as a technical partner that helps me reach that understanding while working
on a project that would otherwise be difficult to approach alone.

**Lesson:** AI assistance is most valuable when it supports learning and
decision-making rather than silently replacing them.

## Conclusion

The common pattern behind these lessons is learning where to spend complexity.
I did not need to design every mechanical part, simulation model, controller,
or development tool from scratch. I needed to understand each layer well enough
to make an informed decision about whether to build it, reuse it, or isolate it.

Those decisions have gradually produced a clearer project architecture:

```text
Gazebo X500
    -> PX4 flight control
    -> ROS 2 Offboard commands
    -> 3D LiDAR sensing
    -> mapping and SLAM
```

The detours were not separate from the project. They were part of learning how
to build it.
