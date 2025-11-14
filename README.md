> Call me old fashioned...

# LxROS

LxROS is a lightweight helper library that simplifies common ROS 2 patterns in **C++** and **Python**.  
It provides small wrappers for node creation, publishers, subscribers, timers, services, actions, and launch files.  
The goal is to reduce boilerplate while keeping ROS 2 concepts explicit and visible. Tries to keep things similar to how they were done in ROS1...

---

Made at LxViral (Lisbon Vision, Robotics, and Learning Lab @FCULisboa)

---

## Overview

This repository contains a single ROS 2 package named `lxros`.  
It installs:

- C++ headers under `include/lxros/`
- Python modules under `python/lxros/`
- Launch helpers (Python), and later optional CLI tools

This initial version contains only the project structure; the helper API will be added incrementally.

## Using LxROS in Your Packages

### C++ Packages

Add in package.xml:

```<depend>lxros</depend>```

Add in CMakeLists.txt:
```find_package(lxros REQUIRED)```

Include in code:

```#include <lxros/lxros.hpp>```


### Python Packages

Add in package.xml:

```<exec_depend>lxros</exec_depend>```

Import in Python:

```import lxros```
