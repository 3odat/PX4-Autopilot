Paste your rich text content here. You can paste directly from

## **Step 1: Install and Set Up Isaac Sim**

### **1️⃣ Install Isaac Sim from ZIP**

If you downloaded the ZIP version, follow these steps:

🔹 **Extract Isaac Sim ZIP**


`unzip isaac_sim-<version>.zip -d ~/omniverse`

🔹 **Go into the Isaac Sim folder**


`cd ~/omniverse/isaac_sim`

🔹 **Run the setup script**

`./setup.sh`

🔹 **Launch Isaac Sim**

`./isaac-sim.sh`

🎯 **Expected Result:**  
Isaac Sim should launch **in a new window**.

* * *

### **2️⃣ Verify NVIDIA GPU and Dependencies**

Since Isaac Sim relies on **CUDA + PhysX**, ensure you have the required libraries:


`nvidia-smi  # Check if GPU is recognized nvcc --version  # Check if CUDA is installed`

🎯 **Expected Result:**  
You should see **GPU details and CUDA version.**

Word or other rich text sources.
